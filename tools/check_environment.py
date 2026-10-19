#!/usr/bin/env python3
"""Read-only dependency checks; never connect to or move hardware."""
import argparse
import ctypes
import importlib
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["all", "hands", "arms"], default="all")
    args = parser.parse_args()
    errors = []
    modules = ["rclpy", "yaml", "numpy", "scipy", "PyQt5.QtWidgets"]
    if args.mode != "arms":
        modules += ["pinocchio", "nlopt", "wuji_retargeting"]
    if args.mode != "hands":
        modules += ["openvr"]
    for name in modules:
        try:
            importlib.import_module(name)
            print(f"OK import {name}")
        except Exception as exc:
            errors.append(f"{name}: {exc}")
    libs = []
    if args.mode != "arms":
        libs.append(ROOT / "src/input_devices/manus_input/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so")
    if args.mode != "hands":
        libs += list((ROOT / "src/output_devices/tianji_output/tianji_output/_internal/lib").glob("*.so"))
        if len(libs) < (2 if args.mode == "arms" else 3):
            errors.append("Tianji shared libraries missing")
    for path in libs:
        try:
            with path.open("rb") as stream:
                if stream.read(4) != b"\x7fELF":
                    raise ValueError("not ELF; run git lfs pull")
            result = subprocess.run(["ldd", str(path)], capture_output=True, text=True)
            if "not found" in result.stdout:
                raise ValueError(result.stdout)
            ctypes.CDLL(str(path))
            print(f"OK SDK {path.name}")
        except Exception as exc:
            errors.append(f"{path}: {exc}")
    for error in errors:
        print(f"FAIL {error}")
    return bool(errors)


if __name__ == "__main__":
    raise SystemExit(main())
