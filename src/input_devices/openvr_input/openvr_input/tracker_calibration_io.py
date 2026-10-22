"""Read project configuration and export Tracker calibration."""

from copy import deepcopy
import math
from pathlib import Path
from tempfile import NamedTemporaryFile

from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedSeq


SIDES = ("left", "right")
QUATERNION_KEYS = ("qx", "qy", "qz", "qw")
PROFILE_KEYS = ("chest_rotation", "hand_back_to_palm", "neutral_pose")


def read_config(path):
    """Resolve installed symlinks and read a YAML mapping."""
    path = Path(path).expanduser().resolve(strict=True)
    with path.open(encoding="utf-8") as stream:
        data = YAML(typ="safe").load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"Expected a YAML mapping: {path}")
    return data


def calibration_directory():
    """Locate calibration data independently of the profiles it contains."""
    package_root = Path(__file__).resolve().parents[1]
    if (package_root / "package.xml").is_file():
        return package_root / "calibration"

    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("openvr_input")) / "calibration"


def calibration_path(input_config):
    """Resolve a package calibration filename or an absolute profile path."""
    path = Path(input_config["calibration_file"]).expanduser()
    if not path.is_absolute():
        if path.parent != Path(".") or not path.name:
            raise ValueError("calibration_file must be a filename or an absolute path")
        path = calibration_directory() / path
    return path.resolve(strict=True)


def calibration_output_path(filename):
    """Resolve an output filename inside the fixed package calibration directory."""
    if (
        not filename
        or filename in (".", "..")
        or Path(filename).name != filename
        or "/" in filename
        or "\\" in filename
    ):
        raise ValueError("--output must be a filename, such as tracker-hand-back.yaml")
    return calibration_directory() / filename


def load_selected_profile(input_config):
    """Read the calibration selected by the input configuration."""
    return read_config(calibration_path(input_config))


def load_output_profile(output):
    """Use the output profile, or the input package's defaults for a new file."""
    output = Path(output)
    source = output if output.exists() else calibration_directory() / "tracker.yaml.template"
    return read_config(source)


def load_context(input_config, output):
    return {
        "input_config": input_config,
        "profile": load_output_profile(output),
    }


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


def _vector(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain {length} coordinates")
    for coordinate in value:
        _number(coordinate, name)


def _quaternion(value, name):
    _vector(value, 4, name)
    norm = sum(coordinate ** 2 for coordinate in value)
    if not math.isclose(norm, 1.0, rel_tol=1e-6, abs_tol=1e-6):
        raise ValueError(f"{name} must have unit length")


def _rotation_matrix(matrix, name):
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError(f"{name} must be 3x3")
    for row in matrix:
        for value in row:
            _number(value, name)
    for i in range(3):
        for j in range(3):
            dot = sum(matrix[k][i] * matrix[k][j] for k in range(3))
            if not math.isclose(dot, float(i == j), abs_tol=1e-6):
                raise ValueError(f"{name} must be orthogonal")
    a, b, c = matrix
    determinant = (a[0] * (b[1] * c[2] - b[2] * c[1])
                   - a[1] * (b[0] * c[2] - b[2] * c[0])
                   + a[2] * (b[0] * c[1] - b[1] * c[0]))
    if not math.isclose(determinant, 1.0, abs_tol=1e-6):
        raise ValueError(f"{name} must have determinant +1")


def validate_profile(profile):
    """Check generated calibration fields before export."""
    if not isinstance(profile, dict):
        raise ValueError("Expected a Tracker calibration mapping")
    try:
        _rotation_matrix(profile["chest_rotation"], "chest_rotation")
        hand_back_to_palm = profile["hand_back_to_palm"]
        neutral_pose = profile["neutral_pose"]
        for side in SIDES:
            _quaternion(
                [hand_back_to_palm[side][key] for key in QUATERNION_KEYS],
                f"hand_back_to_palm.{side}",
            )
            _vector(hand_back_to_palm[side]["tracker_offset_m"], 3,
                    f"hand_back_to_palm.{side}.tracker_offset_m")
            neutral = neutral_pose[side]
            _vector(neutral["position_m"], 3, f"neutral_pose.{side}.position_m")
            for key in ("quaternion_xyzw", "upper_arm_quaternion_xyzw"):
                _quaternion(neutral[key], f"neutral_pose.{side}.{key}")
    except (KeyError, TypeError) as exc:
        raise ValueError(f"Incomplete Tracker calibration: {exc}") from exc
    return profile


def make_profile(result):
    profile = {key: deepcopy(result["calibration"][key]) for key in PROFILE_KEYS}
    return validate_profile(profile)


def _flow_vectors(value):
    if isinstance(value, dict):
        return {key: _flow_vectors(item) for key, item in value.items()}
    if isinstance(value, list):
        sequence = CommentedSeq(_flow_vectors(item) for item in value)
        if not any(isinstance(item, (dict, list)) for item in value):
            sequence.fa.set_flow_style()
        return sequence
    return value


def save_profile(path, profile):
    """Replace the selected profile after its complete YAML has been written."""
    validate_profile(profile)
    path = Path(path).expanduser().absolute()
    yaml = YAML()
    yaml.default_flow_style = False
    yaml.indent(mapping=2, sequence=4, offset=2)
    yaml.width = 4096
    temporary = None
    try:
        with NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent,
            prefix=f".{path.name}.", delete=False,
        ) as stream:
            temporary = Path(stream.name)
            yaml.dump(_flow_vectors(profile), stream)
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return path
