"""Read, export, and apply Tracker calibration without opening devices."""

from copy import deepcopy
from io import StringIO
import math
import os
from pathlib import Path
import tempfile

from ruamel.yaml import YAML


SIDES = ("left", "right")
CHEST_KEYS = ("roll", "pitch", "yaw", "local_x", "local_y", "local_z")
WRIST_KEYS = ("x", "y", "z", "qx", "qy", "qz", "qw")


def read_config(path):
    """Resolve installed symlinks and read a YAML mapping."""
    path = Path(path).expanduser().resolve(strict=True)
    with path.open(encoding="utf-8") as stream:
        data = YAML(typ="safe").load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"Expected a YAML mapping: {path}")
    return data


def default_config(package, filename):
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory(package)) / "config" / filename


def load_context(tracker_path, static_path, robot_path, kinematics_path):
    return {
        "tracker_config": read_config(tracker_path),
        "static_config": read_config(static_path),
        "robot_config": read_config(robot_path),
        "kinematics_path": str(Path(kinematics_path).expanduser().resolve(strict=True)),
    }


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


def validate_profile(profile):
    """Check the data needed to write the existing configuration fields."""
    if not isinstance(profile, dict) or profile.get("schema_version") != 1:
        raise ValueError("Expected a Tracker calibration with schema_version: 1")
    try:
        calibration = profile["calibration"]
        chest = calibration["chest_mount"]
        wrist = calibration["wrist_to_tianji"]
        upper = calibration["upper_arm_rotations"]
        for axis in ("x", "y", "z"):
            _number(chest["shared"][axis], f"chest_mount.shared.{axis}")
        for side in SIDES:
            for key in CHEST_KEYS:
                _number(chest[side][key], f"chest_mount.{side}.{key}")
            for key in WRIST_KEYS:
                _number(wrist[side][key], f"wrist_to_tianji.{side}.{key}")
            norm = sum(wrist[side]["q" + axis] ** 2 for axis in ("x", "y", "z", "w"))
            if not math.isclose(norm, 1.0, rel_tol=1e-6, abs_tol=1e-6):
                raise ValueError(f"{side} wrist quaternion must have unit length")
            matrix = upper[side + "_arm"]
            if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
                raise ValueError(f"{side} upper-arm rotation must be 3x3")
            for row in matrix:
                for value in row:
                    _number(value, f"upper_arm_rotations.{side}_arm")
            for i in range(3):
                for j in range(3):
                    dot = sum(matrix[k][i] * matrix[k][j] for k in range(3))
                    if not math.isclose(dot, float(i == j), abs_tol=1e-6):
                        raise ValueError(f"{side} upper-arm rotation must be orthogonal")
            a, b, c = matrix
            determinant = (a[0] * (b[1] * c[2] - b[2] * c[1])
                           - a[1] * (b[0] * c[2] - b[2] * c[0])
                           + a[2] * (b[0] * c[1] - b[1] * c[0]))
            if not math.isclose(determinant, 1.0, abs_tol=1e-6):
                raise ValueError(f"{side} upper-arm rotation must have determinant +1")
    except (KeyError, TypeError) as exc:
        raise ValueError(f"Incomplete Tracker calibration: {exc}") from exc
    return profile


def make_profile(result, tracker_config):
    profile = {
        "schema_version": 1,
        "reference": deepcopy(result["reference"]),
        "calibration": deepcopy(result["calibration"]),
    }
    profile["reference"]["tracker_serials"] = deepcopy(tracker_config["tracker_serials"])
    profile["reference"]["wrist_offset_m"] = deepcopy(
        tracker_config.get("wrist_offset") or [0.0, 0.0, 0.0])
    if "fit" in result:
        profile["fit"] = deepcopy(result["fit"])
    return validate_profile(profile)


def save_profile(path, profile):
    """Export a new profile at the explicitly selected path."""
    validate_profile(profile)
    path = Path(path).expanduser().absolute()
    yaml = YAML()
    yaml.default_flow_style = False
    with path.open("x", encoding="utf-8") as stream:
        yaml.dump(profile, stream)
    return path


def load_profile(path):
    return validate_profile(read_config(path))


def _merge_fields(target, source, keys):
    for key in keys:
        target[key] = deepcopy(source[key])


def _stage_file(path, content):
    fd, name = tempfile.mkstemp(prefix="." + path.name + ".", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(content)
        os.chmod(temporary, path.stat().st_mode & 0o777)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return temporary


def apply_profile(profile, tracker_path, static_path):
    """Update only calibration fields, preserving unrelated values and comments."""
    validate_profile(profile)
    tracker_path = Path(tracker_path).expanduser().resolve(strict=True)
    static_path = Path(static_path).expanduser().resolve(strict=True)
    if tracker_path == static_path:
        raise ValueError("Tracker and static TF configurations must be different files")
    yaml = YAML()
    yaml.preserve_quotes = True
    with tracker_path.open(encoding="utf-8") as stream:
        tracker = yaml.load(stream)
    with static_path.open(encoding="utf-8") as stream:
        static = yaml.load(stream)
    calibration = profile["calibration"]
    _merge_fields(static["chest_mount"]["shared"],
                  calibration["chest_mount"]["shared"], ("x", "y", "z"))
    for side in SIDES:
        _merge_fields(static["chest_mount"][side],
                      calibration["chest_mount"][side], CHEST_KEYS)
        _merge_fields(static["wrist_to_tianji"][side],
                      calibration["wrist_to_tianji"][side], WRIST_KEYS)
    upper = tracker.setdefault("upper_arm_rotations", {})
    _merge_fields(upper, calibration["upper_arm_rotations"], ("left_arm", "right_arm"))

    # Render both documents before replacing either, without retaining temp files.
    staged = []
    originals = {path: path.read_bytes() for path in (static_path, tracker_path)}
    applied = []
    try:
        for path, document in ((static_path, static), (tracker_path, tracker)):
            text = StringIO()
            yaml.dump(document, text)
            staged.append((path, _stage_file(path, text.getvalue().encode("utf-8"))))
        for path, temporary in staged:
            os.replace(temporary, path)
            applied.append(path)
    except BaseException:
        for path in reversed(applied):
            rollback = _stage_file(path, originals[path])
            try:
                os.replace(rollback, path)
            finally:
                rollback.unlink(missing_ok=True)
        raise
    finally:
        for _, temporary in staged:
            temporary.unlink(missing_ok=True)
    return static_path, tracker_path
