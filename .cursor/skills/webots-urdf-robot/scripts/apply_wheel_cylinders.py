#!/usr/bin/env python3
"""将四个轮胎 boundingObject 改为 Cylinder，Pose 与视觉相同（T1 + 绕 X 轴 90°）。"""
import re
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[4]
R, H = 0.4475, 0.215
ROT = "1 0 0 1.570796"

WHEELS = {
    "link_004": (-0.900988, 0.280422, -0.161446),
    "link_007": (0.36631, 0.280422, -0.161446),
    "link_010": (0.36631, 0.280422, 1.326554),
    "link_013": (-0.807213, 0.280422, 1.326554),
}


def patch_file(path: Path, indent: str) -> None:
    text = path.read_text()
    for link, t1 in WHEELS.items():
        block = (
            f"{indent}boundingObject Pose {{\n"
            f"{indent}  translation {t1[0]} {t1[1]} {t1[2]}\n"
            f"{indent}  rotation {ROT}\n"
            f"{indent}  children [\n"
            f"{indent}    Cylinder {{\n"
            f"{indent}      radius {R:.4f}\n"
            f"{indent}      height {H:.4f}\n"
            f"{indent}    }}\n"
            f"{indent}  ]\n"
            f"{indent}}}"
        )
        pat = re.compile(
            rf'name "{link}"\s+boundingObject Pose \{{[\s\S]*?\n{re.escape(indent)}\}}',
            re.MULTILINE,
        )
        text, n = pat.subn(f'name "{link}"\n{block}', text, count=1)
        if n != 1:
            print(f"Warning: {path.name} {link} not updated", file=sys.stderr)
    path.write_text(text)
    print(f"Updated {path}")


def main() -> int:
    patch_file(PROJECT / "worlds" / "my_project.wbt", "                    ")
    patch_file(PROJECT / "protos" / "robot.proto", "                      ")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
