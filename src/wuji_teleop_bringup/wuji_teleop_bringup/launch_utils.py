"""Resolve local hardware configuration installed by colcon."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def resolve_config_path(package: str, config_file: str) -> str:
    path = Path(get_package_share_directory(package)) / "config" / config_file
    if not path.is_file():
        raise FileNotFoundError(f"Missing {path}; run tools/configure.py before building")
    return str(path)
