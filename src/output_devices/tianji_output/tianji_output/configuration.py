"""Load the arm configuration shared by teleoperation, Home, and calibration."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def arm_config_path(path=None):
    if path:
        return Path(path).expanduser().resolve()
    return (
        Path(get_package_share_directory("tianji_output"))
        / "config" / "tianji_chest.yaml"
    ).resolve()


def load_arm_config(path=None):
    with arm_config_path(path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def kinematics_path(config):
    path = Path(config["driver"]["kinematics_model"]).expanduser()
    if not path.is_absolute():
        path = Path(get_package_share_directory("tianji_output")) / "config" / path
    return path.resolve()
