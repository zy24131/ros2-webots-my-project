#!/usr/bin/env python3
"""Wrapper — use .cursor/skills/resume-pdf/scripts/generate_resume_pdf.py instead."""

import runpy
import sys
from pathlib import Path

SCRIPT = (
    Path(__file__).resolve().parents[1]
    / ".cursor/skills/resume-pdf/scripts/generate_resume_pdf.py"
)
sys.argv[0] = str(SCRIPT)
runpy.run_path(str(SCRIPT), run_name="__main__")
