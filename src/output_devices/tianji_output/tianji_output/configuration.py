"""Load the arm configuration shared by teleoperation, Home, and calibration."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def config_directory():
    """Locate arm configuration without relying on individual file symlinks."""
    package_dir = Path(__file__).resolve().parent
    if (package_dir.parent / "package.xml").is_file():
        return package_dir / "config"
    return Path(get_package_share_directory("tianji_output")) / "config"


def arm_config_path(path=None):
    if path:
        return Path(path).expanduser().resolve()
    return (config_directory() / "tianji_chest.yaml").resolve()


def load_arm_config(path=None):
    with arm_config_path(path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def kinematics_path(config):
    path = Path(config["driver"]["kinematics_model"]).expanduser()
    if not path.is_absolute():
        path = config_directory() / path
    return path.resolve()
