---
name: resume-pdf
description: >-
  Generate styled Chinese resume PDF from resume_data.yaml using ReportLab.
  Red bold section headers, red divider lines, Microsoft YaHei font (with fallback),
  bold project titles and university names. Use when the user asks to generate,
  update, or export a resume/CV PDF (简历, 个人简历, generate resume).
---

# 简历 PDF 生成

## 快速使用

1. 编辑内容 SSOT：[resume_data.yaml](resume_data.yaml)
2. （推荐）复制微软雅黑到 [fonts/](fonts/)：`msyh.ttc`、`msyhbd.ttc`（见 [fonts/README.md](fonts/README.md)）
3. 生成 PDF（ReportLab，行距/缩进稳定）：

```bash
python3 .cursor/skills/resume-pdf/scripts/generate_resume_pdf.py
```

依赖：`python3-yaml`、`python3-reportlab`

## 版式（参考 `徐俊鸣_原版.pdf`）

- 章节标题：红色加粗 + 红色下划线
- 学校/项目名称 **与时间同一行**（左 bold + 右对齐时间）
- 项目子模块：小标题加粗 + **编号列表**（1. 2. 3.）
- 智能动力底盘类长段落：连续段落（非 bullet）
- PINNS 类：纯文本行
- 技能/论文：`标签：` 后接正文（原版格式）

| 元素 | 样式 |
|------|------|
| 章节标题（个人总结、教育背景、项目经历、技能/证书及其他、论文期刊、荣誉奖项） | **红色 `#C00000` + 加粗** |
| 章节标题下横线 | **红色**，厚度 0.8pt |
| 正文字体 | **Microsoft YaHei（微软雅黑）** 字体栈；LibreOffice 自动 fallback 到 Noto Sans CJK SC |
| 项目名称 | **CSS `font-weight:bold` + 11pt**（`.proj-title`） |
| 毕业院校 | **CSS `font-weight:bold` + 10pt**（`.school`） |
| 正文加粗标签 | `<span class="label">` 或对应 class，**禁止**依赖 ReportLab `<b>` |

## 修改内容

只改 [resume_data.yaml](resume_data.yaml)，不要硬编码到脚本：

- `header`：姓名、联系方式、求职意向
- `summary`：个人总结
- `education[]`：`school`（加粗显示）、`period`、`detail`
- `projects[]`：`title`（加粗）、`period`、`role`、`intro`、`blocks[]`
- `skills` / `publications` / `awards`

## 新增项目条目模板

```yaml
  - title: 项目名称（会加粗）
    period: 2025年01月 - 2025年06月
    role: 学生负责人
    intro: 一段总述（可为空字符串）
    blocks:
      - subtitle: 子模块标题（可为空）
        items:
          - 要点一
          - 要点二
```

## 故障排查

**PDF 出现方块 □**：确认已安装 `libreoffice-writer`；检查 HTML 是否 utf-8。

**加粗不明显**：确认使用 LibreOffice 引擎（脚本输出 `Engine: LibreOffice`）；复制 `msyh.ttc`/`msyhbd.ttc` 到 `fonts/` 可改用微软雅黑。

**内容未更新**：确认编辑的是 `resume_data.yaml` 而非旧版 `docs/generate_resume_pdf.py`。

## 文件结构

```text
.cursor/skills/resume-pdf/
├── SKILL.md
├── resume_data.yaml      # 内容 SSOT
├── fonts/                # 放置 msyh.ttc / msyhbd.ttc
└── scripts/
    └── generate_resume_pdf.py
```
