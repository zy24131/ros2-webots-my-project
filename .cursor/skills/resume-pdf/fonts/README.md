# 微软雅黑字体（可选）

将 Windows 字体复制到本目录后，生成的 PDF 会使用 **Microsoft YaHei（微软雅黑）**：

```bash
# WSL / 双系统示例
cp /mnt/c/Windows/Fonts/msyh.ttc  .cursor/skills/resume-pdf/fonts/
cp /mnt/c/Windows/Fonts/msyhbd.ttc .cursor/skills/resume-pdf/fonts/
```

若未放置上述文件，脚本会使用 Linux 可用的备选字体（AR PL UKai），并在终端提示。
