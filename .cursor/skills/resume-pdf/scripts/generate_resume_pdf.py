#!/usr/bin/env python3
"""Generate resume PDF from resume_data.yaml (ReportLab — stable spacing/indent)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT, TA_RIGHT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import cm, mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import HRFlowable, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

SKILL_DIR = Path(__file__).resolve().parents[1]
DEFAULT_DATA = SKILL_DIR / "resume_data.yaml"
SECTION_RED = colors.HexColor("#C00000")
FALLBACK_REG = "/usr/share/fonts/truetype/arphic/uming.ttc"
FALLBACK_BOLD = "/usr/share/fonts/truetype/arphic/ukai.ttc"
FONT_REG = "YaHeiReg"
FONT_BOLD = "YaHeiBold"
PAGE_W = A4[0] - 3.2 * cm  # content width


def resolve_fonts() -> None:
    candidates = [
        (SKILL_DIR / "fonts" / "msyh.ttc", SKILL_DIR / "fonts" / "msyhbd.ttc"),
        (Path("/mnt/c/Windows/Fonts/msyh.ttc"), Path("/mnt/c/Windows/Fonts/msyhbd.ttc")),
    ]
    reg, bold = FALLBACK_REG, FALLBACK_BOLD
    for r, b in candidates:
        if r.is_file():
            reg = str(r)
            bold = str(b) if b.is_file() else FALLBACK_BOLD
            break
    else:
        print("提示：使用 UMing+UKai 备选字体；复制 msyh.ttc 到 fonts/ 可换微软雅黑", file=sys.stderr)

    kw = {"subfontIndex": 0}
    pdfmetrics.registerFont(TTFont(FONT_REG, reg, **kw))
    pdfmetrics.registerFont(TTFont(FONT_BOLD, bold, **kw if bold.endswith(".ttc") else {}))


def _ps(name, parent, font, **kw):
    return ParagraphStyle(name, parent=parent, fontName=font, wordWrap="CJK", **kw)


def build_styles():
    b = getSampleStyleSheet()
    return {
        "name": _ps("name", b["Normal"], FONT_BOLD, fontSize=15, leading=18, spaceAfter=2),
        "meta": _ps("meta", b["Normal"], FONT_REG, fontSize=10, leading=13, textColor=colors.HexColor("#333")),
        "meta_r": _ps("meta_r", b["Normal"], FONT_REG, fontSize=10, leading=13, alignment=TA_RIGHT),
        "sec": _ps("sec", b["Normal"], FONT_BOLD, fontSize=11, leading=14, textColor=SECTION_RED, spaceBefore=6, spaceAfter=2),
        "body": _ps("body", b["Normal"], FONT_REG, fontSize=10.5, leading=14, alignment=TA_LEFT, spaceAfter=2),
        "role": _ps("role", b["Normal"], FONT_BOLD, fontSize=10.5, leading=14, spaceAfter=2),
        "sub": _ps("sub", b["Normal"], FONT_BOLD, fontSize=10.5, leading=14, spaceBefore=3, spaceAfter=1),
        "num": _ps("num", b["Normal"], FONT_REG, fontSize=10.5, leading=14, leftIndent=14, firstLineIndent=-14, spaceAfter=2),
    }


def split_city_line(line: str) -> tuple[str, str]:
    line = line.strip()
    for sep in ("　", "\t", " "):
        if sep in line:
            left, right = line.rsplit(sep, 1)
            if right in ("北京", "合肥"):
                return left.strip(), right.strip()
    return line, ""


def row_table(left: str, right: str, st, bold_left: bool = True) -> Table:
    lfont = FONT_BOLD if bold_left else FONT_REG
    left_p = Paragraph(f'<font name="{lfont}">{left}</font>', st["body"])
    right_p = Paragraph(right, st["meta_r"]) if right else Paragraph("", st["body"])
    t = Table([[left_p, right_p]], colWidths=[PAGE_W * 0.72, PAGE_W * 0.28])
    t.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 0),
        ("RIGHTPADDING", (0, 0), (-1, -1), 0),
        ("TOPPADDING", (0, 0), (-1, -1), 0),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 1),
    ]))
    return t


def section(st, title):
    return [
        Spacer(1, 2 * mm),
        Paragraph(title, st["sec"]),
        HRFlowable(width="100%", thickness=0.6, color=SECTION_RED),
        Spacer(1, 1.5 * mm),
    ]


def render_block(block: dict, st) -> list:
    flow = []
    subtitle = (block.get("subtitle") or "").strip()
    if subtitle:
        flow.append(Paragraph(subtitle, st["sub"]))
    style = block.get("style", "numbered")
    for i, item in enumerate(block.get("items") or [], 1):
        if style == "numbered":
            flow.append(Paragraph(f"{i}.{item}", st["num"]))
        else:
            flow.append(Paragraph(item, st["body"]))
    return flow


def build_story(data: dict, st) -> list:
    s = []
    h = data["header"]
    s.append(Paragraph(f'<font name="{FONT_BOLD}">{h["name"]}</font>', st["name"]))
    s.append(Paragraph(h["contact"], st["meta"]))
    if h.get("id"):
        s.append(Paragraph(h["id"], st["meta"]))
    s.append(Paragraph(h["intent"], st["meta_r"]))
    s.append(Spacer(1, 2 * mm))

    s.extend(section(st, "个人总结"))
    for line in data["summary"].strip().split("\n"):
        line = line.strip()
        if line:
            s.append(Paragraph(line, st["body"]))

    s.extend(section(st, "教育背景"))
    for i, edu in enumerate(data["education"]):
        s.append(row_table(edu["school"], edu["period"], st, bold_left=True))
        left2, city = split_city_line(edu.get("line2", ""))
        if city:
            s.append(row_table(left2, city, st, bold_left=False))
        elif left2:
            s.append(Paragraph(left2, st["body"]))
        if edu.get("line3"):
            s.append(Paragraph(edu["line3"], st["body"]))
        if i + 1 < len(data["education"]):
            s.append(Spacer(1, 1.5 * mm))

    s.extend(section(st, "项目经历"))
    for proj in data["projects"]:
        s.append(row_table(proj["title"], proj["period"], st, bold_left=True))
        s.append(Paragraph(proj["role"], st["role"]))
        intro = " ".join((proj.get("intro") or "").split())
        if intro:
            s.append(Paragraph(intro, st["body"]))
        for block in proj.get("blocks") or []:
            s.extend(render_block(block, st))
        s.append(Spacer(1, 1.5 * mm))

    s.extend(section(st, "技能/证书及其他"))
    for item in data["skills"]:
        s.append(Paragraph(f'<font name="{FONT_BOLD}">{item["label"]}：</font>{item["text"]}', st["body"]))

    s.extend(section(st, "论文期刊"))
    for pub in data["publications"]:
        s.append(Paragraph(f'<font name="{FONT_BOLD}">{pub["label"]} :</font>{pub["text"]}', st["body"]))

    s.extend(section(st, "荣誉奖项"))
    for award in data["awards"]:
        if isinstance(award, dict):
            s.append(row_table(award["title"], award["period"], st, bold_left=False))
        else:
            s.append(Paragraph(award, st["body"]))
    return s


def generate(data_path: Path, output_path: Path) -> None:
    resolve_fonts()
    with data_path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    st = build_styles()
    doc = SimpleDocTemplate(
        str(output_path), pagesize=A4,
        leftMargin=1.6 * cm, rightMargin=1.6 * cm,
        topMargin=1.4 * cm, bottomMargin=1.4 * cm,
    )
    doc.build(build_story(data, st))
    print(f"Written: {output_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    with args.data.open(encoding="utf-8") as f:
        out = Path(yaml.safe_load(f).get("output", "/home/kemove/Downloads/徐俊鸣.pdf"))
    generate(args.data, args.output or out)


if __name__ == "__main__":
    main()
