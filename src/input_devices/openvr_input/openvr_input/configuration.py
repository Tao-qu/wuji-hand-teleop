"""Load standalone OpenVR input configuration."""

from pathlib import Path

from .tracker_calibration_io import read_config


def config_directory():
    """Locate input settings in the source package or installed share directory."""
    package_root = Path(__file__).resolve().parents[1]
    if (package_root / "package.xml").is_file():
        return package_root / "config"

    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("openvr_input")) / "config"


def input_config_path(path=None):
    """Resolve an explicit input configuration or the bundled openvr.yaml."""
    if path:
        return Path(path).expanduser().resolve()
    return (config_directory() / "openvr.yaml").resolve()


def load_input_config(path=None):
    """Read the top-level input settings selected by ``path``."""
    return read_config(input_config_path(path))
