#!/usr/bin/env python3
"""Create local hardware YAML files without overwriting existing settings."""
import argparse
from pathlib import Path
import shutil


CONFIG_TEMPLATES = (
    "src/output_devices/wujihand2_output/wujihand2_ros2/config/wujihand2.yaml.template",
    "src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml.template",
)


def configure(root, *, check_only=False):
    """Create missing hand and arm configurations, excluding fetched sources."""
    root = Path(root)
    created = []
    for relative in CONFIG_TEMPLATES:
        template = root / relative
        if not template.is_file():
            raise FileNotFoundError(f"Missing configuration template: {template}")
        destination = template.with_suffix("")
        if destination.exists() and not destination.is_file():
            raise ValueError(f"Local configuration is not a file: {destination}")
        if destination.is_symlink() and not destination.exists():
            raise ValueError(f"Local configuration symlink is broken: {destination}")
        if not destination.exists():
            if not check_only:
                shutil.copyfile(template, destination)
            created.append(destination)
    return created


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Report missing local files without writing")
    args = parser.parse_args()
    paths = configure(Path(__file__).resolve().parents[1], check_only=args.check)
    for path in paths:
        print(f"{'Missing' if args.check else 'Created'} {path}")
    if args.check and not paths:
        print("All local configuration files exist")
    raise SystemExit(1 if args.check and paths else 0)
