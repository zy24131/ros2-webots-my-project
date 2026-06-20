#!/usr/bin/env python3
"""Scale binary STL files in meshes/ by 0.001 (mm to meters)."""
import struct
import sys
from pathlib import Path

SCALE = 0.001
PROJECT = Path(__file__).resolve().parents[4]
MESH_DIR = PROJECT / "meshes"


def scale_stl(path: Path, scale: float) -> None:
    data = bytearray(path.read_bytes())
    if data[:5].lower() == b"solid":
        raise ValueError(f"ASCII STL not supported: {path.name}")
    tri_count = struct.unpack_from("<I", data, 80)[0]
    offset = 84
    record_size = 50
    for i in range(tri_count):
        base = offset + i * record_size + 12
        for v in range(3):
            vi = base + v * 12
            x, y, z = struct.unpack_from("<fff", data, vi)
            struct.pack_into("<fff", data, vi, x * scale, y * scale, z * scale)
    path.write_bytes(data)


def main() -> int:
    if not MESH_DIR.is_dir():
        print(f"Missing mesh dir: {MESH_DIR}", file=sys.stderr)
        return 1
    for stl in sorted(MESH_DIR.glob("*.stl")):
        scale_stl(stl, SCALE)
        print(f"Scaled {stl.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
