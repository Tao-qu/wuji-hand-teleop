"""Home-reference kinematics and mirror-constrained Tracker calibration."""

from copy import deepcopy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


SIDES = ("left", "right")
TRACKER_MIRROR = np.diag([-1.0, 1.0, 1.0])
ARM_MIRROR = np.diag([1.0, -1.0, 1.0])


def home_reference(context):
    """Calculate the configured Home TCP and upper-arm directions locally."""
    # Load the native SDK only when calculating the Home reference.
    from tianji_output._internal.fx_kine import Marvin_Kine
    from tianji_output.tool_parameters import tool_transform

    robot_config = context["robot_config"]
    model_path = str(context["kinematics_path"])
    kine = Marvin_Kine()
    model = kine.load_config(config_path=model_path)
    if model is None:
        raise RuntimeError(f"Cannot load kinematics model: {model_path}")

    reference = {
        "kinematics_model": Path(model_path).name,
        "init_joints": {},
        "home_tcp": {},
        "upper_arm_directions": {},
        "tool_transforms": {},
        "units": {
            "init_joints": "degrees",
            "home_tcp_translation": "metres",
            "tool_transforms_translation": "millimetres",
            "upper_arm_directions": "unit vectors",
        },
    }
    for index, side in enumerate(SIDES):
        joints = [float(value) for value in robot_config["init_joints"][side]]
        tool = tool_transform(robot_config, side)
        if not kine.initial_kine(
            robot_serial=index,
            robot_type=model["TYPE"][index],
            dh=model["DH"][index],
            pnva=model["PNVA"][index],
            j67=model["BD"][index],
        ):
            raise RuntimeError(f"Cannot initialize {side} Home kinematics")
        if not kine.set_tool_kine(index, tool):
            raise RuntimeError(f"Cannot set {side} Home tool geometry")
        result = kine.fk(index, joints)
        if result is False or result is None:
            raise RuntimeError(f"Cannot calculate {side} Home FK")
        tcp = np.asarray(result, dtype=float)
        tcp[:3, 3] /= 1000.0

        # Existing M6 zsp_type=1 uses the negative upper-arm direction.
        q1, q2 = np.deg2rad(joints[:2])
        direction = np.array([
            np.sin(q2) * np.cos(q1),
            np.sin(q2) * np.sin(q1),
            -np.cos(q2),
        ]).tolist()
        reference["init_joints"][side] = joints
        reference["home_tcp"][side] = tcp.tolist()
        reference["upper_arm_directions"][side] = direction
        reference["tool_transforms"][side] = tool
    return reference


def _base_rotation(settings):
    # Match tf_utils: legacy YAML roll/pitch/yaw means CLI yaw/pitch/roll.
    return Rotation.from_euler(
        "xyz", [settings["yaw"], settings["pitch"], settings["roll"]]
    ).as_matrix()


def _rotation_from_y(direction):
    """Return the shortest proper rotation taking local Y to direction."""
    length = np.linalg.norm(direction)
    if length == 0.0:
        raise ValueError("The mean upper-arm reference direction is undefined")
    target = direction / length
    source = np.array([0.0, 1.0, 0.0])
    cross = np.cross(source, target)
    sine = np.linalg.norm(cross)
    cosine = float(np.dot(source, target))
    if sine <= np.finfo(float).eps:
        return np.eye(3) if cosine >= 0.0 else np.diag([1.0, -1.0, -1.0])
    return Rotation.from_rotvec(cross / sine * np.arctan2(sine, cosine)).as_matrix()


def fit_calibration(context, samples):
    """Fit synchronized raw world poses; return fields, residuals and reference.

    ``samples`` maps chest, left/right_wrist and left/right_arm to (N, 4, 4)
    arrays. Capture owns device validity and synchronization. This function
    performs no device acquisition, configuration writes or residual gating.
    """
    from .openvr_tracker_wrapper import OpenVRTrackerWrapper

    reference = home_reference(context)
    tracker_config = context["tracker_config"]
    static_config = context["static_config"]
    chest_settings = static_config["chest_mount"]
    wrist_settings = static_config["wrist_to_tianji"]
    raw_chest = np.asarray(samples["chest"], dtype=float)
    count = len(raw_chest)
    if count == 0:
        raise ValueError("Calibration requires a synchronized pose sample")

    chest = raw_chest @ OpenVRTrackerWrapper._CHEST_CORRECTION
    chest_inverse_rotation = chest[:, :3, :3].transpose(0, 2, 1)
    shared = np.array([chest_settings["shared"][axis] for axis in ("x", "y", "z")])
    wrist_offset = np.asarray(tracker_config.get("wrist_offset") or [0, 0, 0], dtype=float)
    data = {}

    for side in SIDES:
        raw_wrist = np.asarray(samples[f"{side}_wrist"], dtype=float)
        raw_upper = np.asarray(samples[f"{side}_arm"], dtype=float)
        nominal = getattr(
            OpenVRTrackerWrapper, f"_{side.upper()}_WRIST_CORRECTION"
        )[:3, :3]
        base = _base_rotation(chest_settings[side])
        home = np.asarray(reference["home_tcp"][side])
        home_upper = np.asarray(reference["upper_arm_directions"][side])
        static_offset = np.array([wrist_settings[side][axis] for axis in ("x", "y", "z")])

        # The saved wrist translation is in the nominal corrected wrist frame.
        # Keep it, as well as the raw tracker-local mounting-point offset.
        world_point = raw_wrist[:, :3, 3] + raw_wrist[:, :3, :3] @ (
            wrist_offset + nominal @ static_offset
        )
        chest_point = np.einsum(
            "nij,nj->ni", chest_inverse_rotation, world_point - chest[:, :3, 3]
        )
        base_point = (chest_point - shared) @ base
        wrist_rotation = base.T @ chest_inverse_rotation @ raw_wrist[:, :3, :3]
        upper_rotation = base.T @ chest_inverse_rotation @ raw_upper[:, :3, :3]

        data[side] = {
            "nominal": nominal,
            "home": home,
            "home_upper": home_upper,
            "base_point": base_point,
            "wrist_rotation": wrist_rotation,
            "upper_rotation": upper_rotation,
            "local_candidates": base_point - home[:3, 3],
            "wrist_candidates": wrist_rotation.transpose(0, 2, 1) @ home[:3, :3],
            "upper_candidates": upper_rotation.transpose(0, 2, 1) @ home_upper,
        }

    left, right = data["left"], data["right"]
    local_left = np.concatenate(
        [left["local_candidates"], right["local_candidates"] @ ARM_MIRROR]
    ).mean(axis=0)
    raw_to_target_left = Rotation.from_matrix(np.concatenate([
        left["wrist_candidates"],
        TRACKER_MIRROR @ right["wrist_candidates"] @ ARM_MIRROR,
    ])).mean().as_matrix()
    upper_direction_left = np.concatenate(
        [left["upper_candidates"], right["upper_candidates"] @ TRACKER_MIRROR]
    ).mean(axis=0)
    upper_left = _rotation_from_y(upper_direction_left)

    local = {"left": local_left, "right": ARM_MIRROR @ local_left}
    raw_to_target = {
        "left": raw_to_target_left,
        "right": TRACKER_MIRROR @ raw_to_target_left @ ARM_MIRROR,
    }
    upper = {"left": upper_left, "right": TRACKER_MIRROR @ upper_left @ TRACKER_MIRROR}
    calibration = {
        "chest_mount": deepcopy(chest_settings),
        "wrist_to_tianji": deepcopy(wrist_settings),
        "upper_arm_rotations": deepcopy(tracker_config.get("upper_arm_rotations") or {}),
    }
    fit = {"sample_count": count}

    for side in SIDES:
        current = data[side]
        for axis, value in zip(("x", "y", "z"), local[side]):
            calibration["chest_mount"][side][f"local_{axis}"] = float(value)
        # Runtime already applies nominal wrist correction before this static TF.
        quaternion = Rotation.from_matrix(
            current["nominal"].T @ raw_to_target[side]
        ).as_quat()
        if quaternion[3] < 0:
            quaternion = -quaternion
        for key, value in zip(("qx", "qy", "qz", "qw"), quaternion):
            calibration["wrist_to_tianji"][side][key] = float(value)
        calibration["upper_arm_rotations"][f"{side}_arm"] = upper[side].tolist()

        position_error = np.linalg.norm(
            current["base_point"] - local[side] - current["home"][:3, 3], axis=1
        )
        wrist_angle = np.rad2deg(Rotation.from_matrix(
            current["home"][:3, :3].T @ current["wrist_rotation"] @ raw_to_target[side]
        ).magnitude())
        upper_direction = current["upper_rotation"] @ upper[side][:, 1]
        cosine = (upper_direction @ current["home_upper"]) / np.linalg.norm(
            upper_direction, axis=1
        )
        upper_angle = np.rad2deg(np.arccos(np.clip(cosine, -1.0, 1.0)))
        fit[side] = {
            "mean_position_error_mm": float(position_error.mean() * 1000.0),
            "mean_wrist_angle_deg": float(wrist_angle.mean()),
            "mean_upper_direction_angle_deg": float(upper_angle.mean()),
            "position_rms_m": float(np.sqrt(np.mean(position_error ** 2))),
            "orientation_rms_deg": float(np.sqrt(np.mean(wrist_angle ** 2))),
            "upper_arm_rms_deg": float(np.sqrt(np.mean(upper_angle ** 2))),
        }

    return {"calibration": calibration, "fit": fit, "reference": reference}
