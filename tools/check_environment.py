#!/usr/bin/env python3
"""Read-only dependency checks; never connect to or move hardware."""
import argparse
import ctypes
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def check_configuration(mode):
    """Resolve the same settings and calibration files used by the launchers."""
    import yaml
    from ament_index_python.packages import get_package_share_directory

    if mode != "arms":
        from wuji_teleop_bringup.launch_utils import resolve_config_path

        config_path = Path(resolve_config_path("wujihand2_ros2", "wujihand2.yaml"))
        config = yaml.safe_load(config_path.read_text())
        calibration_dir = ROOT / "src/input_devices/manus_input/manus_ros2/calibration"
        if not calibration_dir.is_dir():
            calibration_dir = Path(get_package_share_directory("manus_ros2")) / "calibration"
        for side in ("left", "right"):
            values = config[f"{side}_hand"]
            if len(values["home_joints"]) != 20:
                raise ValueError(f"{side} hand Home must contain 20 positions")
            configured = config.get("manus", {}).get(f"{side}_calibration_file")
            profile = Path(configured or f"{side.title()}MetaglovePro.mcal")
            if not profile.is_absolute():
                profile = calibration_dir / profile
            if not profile.is_file() or profile.stat().st_size == 0:
                raise ValueError(f"Missing MANUS calibration: {profile}")
            print(f"OK {side} MANUS calibration {profile.name}")
        print(f"OK hand configuration {config_path.resolve()}")
    if mode != "hands":
        from tianji_output.configuration import arm_config_path, kinematics_path, load_arm_config
        from openvr_input.tracker_calibration_io import calibration_path, load_selected_profile, validate_profile

        config = load_arm_config()
        for side in ("left", "right"):
            if len(config[f"{side}_arm"]["home_joints"]) != 7:
                raise ValueError(f"{side} arm Home must contain seven positions")
        if not kinematics_path(config).is_file():
            raise ValueError(f"Missing kinematics model: {kinematics_path(config)}")
        validate_profile(load_selected_profile(config))
        print(f"OK arm configuration {arm_config_path()}")
        print(f"OK Tracker calibration {calibration_path(config)}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["all", "hands", "arms"], default="all")
    args = parser.parse_args()
    errors = []
    modules = ["rclpy", "yaml", "numpy", "scipy", "PyQt5.QtWidgets"]
    if args.mode != "arms":
        modules += ["wuji_sdk", "wujihand2_ros2"]
    if args.mode != "hands":
        modules += ["openvr"]
    for name in modules:
        try:
            importlib.import_module(name)
            print(f"OK import {name}")
        except Exception as exc:
            errors.append(f"{name}: {exc}")
    for requirement in (ROOT / "requirements.txt").read_text().splitlines():
        if "==" not in requirement:
            continue
        name, wanted = requirement.strip().split("==", 1)
        try:
            installed = importlib.metadata.version(name)
            if installed != wanted:
                raise ValueError(f"installed {installed}, expected {wanted}")
            print(f"OK version {name}=={wanted}")
        except Exception as exc:
            errors.append(f"{name}: {exc}")
    try:
        check_configuration(args.mode)
    except Exception as exc:
        errors.append(f"Configuration: {exc}")
    if args.mode != "arms":
        try:
            from ament_index_python.packages import get_package_share_directory
            from wuji_sdk import RetargetSession, HandModel, Handedness
            model = Path(get_package_share_directory("wuji_hand2_2_description"))
            for side, handedness in (("left", Handedness.Left), ("right", Handedness.Right)):
                if not (model / "urdf" / f"{side}-ros.urdf").is_file():
                    raise ValueError(f"Missing {side} model")
                RetargetSession.for_hand(HandModel.WujiHand2, side=handedness)
            print("OK Hand 2.2 models and official retarget sessions")
        except Exception as exc:
            errors.append(f"Hand 2 model/retargeting: {exc}")
        try:
            lock = json.loads((ROOT / "dependencies.lock.json").read_text())
            for name, spec in lock.items():
                revision = subprocess.check_output(
                    ["git", "-C", str(ROOT / spec["path"]), "rev-parse", "HEAD"], text=True).strip()
                if revision != spec["commit"]:
                    raise ValueError(f"{name}: revision does not match lock file")
            print("OK locked model source revision")
        except Exception as exc:
            errors.append(f"Source dependencies: {exc}")
    libs = []
    if args.mode != "arms":
        libs.append(ROOT / "src/input_devices/manus_input/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so")
    if args.mode != "hands":
        tianji_libs = ROOT / "src/output_devices/tianji_output/tianji_output/_internal/lib"
        marvin_override = os.environ.get("TIANJI_SDK_LIBRARY")
        libs += [tianji_libs / "libKine.so",
                 Path(marvin_override).expanduser() if marvin_override else tianji_libs / "libMarvinSDK.so"]
        if marvin_override:
            print(f"INFO using local Tianji control SDK override: {Path(marvin_override).expanduser()}")
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
