"""Home-reference kinematics and mirror-constrained Tracker calibration."""

from copy import deepcopy

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

    arm_config = context["arm_config"]
    model_path = str(context["kinematics_path"])
    kine = Marvin_Kine()
    model = kine.load_config(config_path=model_path)
    if model is None:
        raise RuntimeError(f"Cannot load kinematics model: {model_path}")

    reference = {
        "home_tcp": {},
        "upper_arm_directions": {},
    }
    for index, side in enumerate(SIDES):
        joints = [float(value) for value in arm_config[f"{side}_arm"]["home_joints"]]
        tool = tool_transform(arm_config["driver"], side)
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
        reference["home_tcp"][side] = tcp.tolist()
        reference["upper_arm_directions"][side] = direction
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
    """Fit synchronized raw world poses; return fields and console residuals.

    ``samples`` maps chest, left/right_hand_back and left/right_arm to (N, 4, 4)
    arrays. Capture owns device validity and synchronization. This function
    performs no device acquisition, configuration writes or residual gating.
    """
    from .openvr_tracker_wrapper import OpenVRTrackerWrapper

    reference = home_reference(context)
    profile = context["profile"]
    chest_settings = profile["chest_mount"]
    raw_chest = np.asarray(samples["chest"], dtype=float)
    count = len(raw_chest)
    if count == 0:
        raise ValueError("Calibration requires a synchronized pose sample")

    chest = raw_chest @ OpenVRTrackerWrapper._CHEST_CORRECTION
    chest_inverse_rotation = chest[:, :3, :3].transpose(0, 2, 1)
    shared = np.array([chest_settings["shared"][axis] for axis in ("x", "y", "z")])
    data = {}

    for side in SIDES:
        raw_hand_back = np.asarray(samples[f"{side}_hand_back"], dtype=float)
        raw_upper = np.asarray(samples[f"{side}_arm"], dtype=float)
        nominal = getattr(
            OpenVRTrackerWrapper, f"_{side.upper()}_HAND_BACK_CORRECTION"
        )[:3, :3]
        base = _base_rotation(chest_settings[side])
        home = np.asarray(reference["home_tcp"][side])
        home_upper = np.asarray(reference["upper_arm_directions"][side])
        # The human hand-back point is the raw Tracker origin.
        world_point = raw_hand_back[:, :3, 3]
        chest_point = np.einsum(
            "nij,nj->ni", chest_inverse_rotation, world_point - chest[:, :3, 3]
        )
        base_point = (chest_point - shared) @ base
        hand_back_rotation = base.T @ chest_inverse_rotation @ raw_hand_back[:, :3, :3]
        upper_rotation = base.T @ chest_inverse_rotation @ raw_upper[:, :3, :3]

        data[side] = {
            "nominal": nominal,
            "home": home,
            "home_upper": home_upper,
            "base_point": base_point,
            "hand_back_rotation": hand_back_rotation,
            "upper_rotation": upper_rotation,
            "local_candidates": base_point - home[:3, 3],
            "tcp_candidates": hand_back_rotation.transpose(0, 2, 1) @ home[:3, :3],
            "upper_candidates": upper_rotation.transpose(0, 2, 1) @ home_upper,
        }

    left, right = data["left"], data["right"]
    local_left = np.concatenate(
        [left["local_candidates"], right["local_candidates"] @ ARM_MIRROR]
    ).mean(axis=0)
    raw_to_target_left = Rotation.from_matrix(np.concatenate([
        left["tcp_candidates"],
        TRACKER_MIRROR @ right["tcp_candidates"] @ ARM_MIRROR,
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
        "hand_back_to_tcp": {side: {} for side in SIDES},
        "upper_arm_rotations": deepcopy(profile["upper_arm_rotations"]),
    }
    fit = {"sample_count": count}

    for side in SIDES:
        current = data[side]
        for axis, value in zip(("x", "y", "z"), local[side]):
            calibration["chest_mount"][side][f"local_{axis}"] = float(value)
        # Runtime applies nominal hand-back correction before this rotation-only TF.
        quaternion = Rotation.from_matrix(
            current["nominal"].T @ raw_to_target[side]
        ).as_quat()
        if quaternion[3] < 0:
            quaternion = -quaternion
        for key, value in zip(("qx", "qy", "qz", "qw"), quaternion):
            calibration["hand_back_to_tcp"][side][key] = float(value)
        calibration["upper_arm_rotations"][f"{side}_arm"] = upper[side].tolist()

        position_error = np.linalg.norm(
            current["base_point"] - local[side] - current["home"][:3, 3], axis=1
        )
        tcp_angle = np.rad2deg(Rotation.from_matrix(
            current["home"][:3, :3].T @ current["hand_back_rotation"] @ raw_to_target[side]
        ).magnitude())
        upper_direction = current["upper_rotation"] @ upper[side][:, 1]
        cosine = (upper_direction @ current["home_upper"]) / np.linalg.norm(
            upper_direction, axis=1
        )
        upper_angle = np.rad2deg(np.arccos(np.clip(cosine, -1.0, 1.0)))
        fit[side] = {
            "mean_position_error_mm": float(position_error.mean() * 1000.0),
            "mean_tcp_angle_deg": float(tcp_angle.mean()),
            "mean_upper_direction_angle_deg": float(upper_angle.mean()),
            "position_rms_m": float(np.sqrt(np.mean(position_error ** 2))),
            "orientation_rms_deg": float(np.sqrt(np.mean(tcp_angle ** 2))),
            "upper_arm_rms_deg": float(np.sqrt(np.mean(upper_angle ** 2))),
        }

    return {"calibration": calibration, "fit": fit}
