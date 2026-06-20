#!/usr/bin/env python3
"""打印四轮 boundingObject Pose（与视觉相同：T1 + 绕 X 轴 90°）。"""
import sys
from pathlib import Path

WHEELS = {
    "link_004": ("part_007_NAUO7.stl", (-0.900988, 0.280422, -0.161446)),
    "link_007": ("part_004_NAUO4.stl", (0.36631, 0.280422, -0.161446)),
    "link_010": ("part_012_NAUO12.stl", (0.36631, 0.280422, 1.326554)),
    "link_013": ("part_013_NAUO13.stl", (-0.807213, 0.280422, 1.326554)),
}


def main() -> int:
    for link, (stl, t1) in WHEELS.items():
        print(f"{link} ({stl}):")
        print(f"  translation {t1[0]} {t1[1]} {t1[2]}")
        print("  rotation 1 0 0 1.570796  # 绕 X 轴 90°，与视觉一致")
        print("  Cylinder { radius 0.4475 height 0.2150 }  # 或 Mesh")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
