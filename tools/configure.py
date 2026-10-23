#!/usr/bin/env python3
"""Create local configuration and calibration files without overwriting existing files."""
import argparse
from pathlib import Path
import shutil


TELEOP_TEMPLATES = (
    "src/output_devices/wujihand2_output/wujihand2_ros2/config/wujihand2.yaml.template",
    "src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml.template",
    "src/input_devices/openvr_input/config/openvr.yaml.template",
    "src/input_devices/manus_input/manus_ros2/calibration/LeftMetaglovePro.mcal.template",
    "src/input_devices/manus_input/manus_ros2/calibration/RightMetaglovePro.mcal.template",
    "src/input_devices/openvr_input/calibration/tracker.yaml.template",
    "src/data_collection/config/recording.yaml.template",
)
CAMERA_TEMPLATES = (
    "src/camera/camera_bringup/config/cameras.yaml.template",
    *(f"src/camera/camera_bringup/calibration/{name}.yaml.template"
      for name in ("head_left", "head_right", "left_wrist", "right_wrist")),
)
CONFIG_TEMPLATES = TELEOP_TEMPLATES + CAMERA_TEMPLATES


def configure(root, *, check_only=False, profile="all"):
    """Create missing device settings and input profiles from their templates."""
    root = Path(root)
    created = []
    templates = {"teleop": TELEOP_TEMPLATES, "camera": CAMERA_TEMPLATES,
                 "all": CONFIG_TEMPLATES}[profile]
    for relative in templates:
        template = root / relative
        if not template.is_file():
            raise FileNotFoundError(f"Missing template: {template}")
        destination = template.with_suffix("")
        if destination.exists() and not destination.is_file():
            raise ValueError(f"Local file is not a regular file: {destination}")
        if destination.is_symlink() and not destination.exists():
            raise ValueError(f"Local file symlink is broken: {destination}")
        if not destination.exists():
            if not check_only:
                shutil.copyfile(template, destination)
            created.append(destination)
    return created


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Report missing local files without writing")
    parser.add_argument("--profile", choices=("teleop", "camera", "all"), default="all",
                        help="Select configuration templates (default: all)")
    args = parser.parse_args()
    paths = configure(Path(__file__).resolve().parents[1], check_only=args.check, profile=args.profile)
    for path in paths:
        print(f"{'Missing' if args.check else 'Created'} {path}")
    if args.check and not paths:
        print("All local configuration and calibration files exist")
    raise SystemExit(1 if args.check and paths else 0)
