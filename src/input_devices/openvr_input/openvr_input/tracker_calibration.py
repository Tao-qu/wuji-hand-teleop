"""Mirror-constrained Tracker calibration in human chest and palm coordinates."""

from copy import deepcopy

import numpy as np
from scipy.spatial.transform import Rotation


SIDES = ("left", "right")
TRACKER_MIRROR = np.diag([-1.0, 1.0, 1.0])
BODY_MIRROR = np.diag([1.0, -1.0, 1.0])


def hand_back_to_palm_transform(profile, side):
    """Return the raw hand-back Tracker -> human palm transform.

    Both palms use X toward the fingers, Z toward the dorsum, and Y = Z cross X.
    ``tracker_offset_m`` points from the palm origin to the Tracker origin,
    expressed in these palm axes. No nominal rotation precedes this transform.
    """
    alignment = profile["hand_back_to_palm"][side]
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(
        [alignment[key] for key in ("qx", "qy", "qz", "qw")]
    ).as_matrix()
    transform[:3, 3] = -transform[:3, :3] @ np.asarray(
        alignment["tracker_offset_m"], dtype=float
    )
    return transform


def _quaternion(matrix):
    quaternion = Rotation.from_matrix(matrix).as_quat()
    if quaternion[3] < 0:
        quaternion = -quaternion
    return quaternion.tolist()


def _paired_rotations(left, right, local_mirror):
    """Average two sides in the left frame, then mirror the shared result."""
    mean = Rotation.from_matrix(np.concatenate([
        left, BODY_MIRROR @ right @ local_mirror,
    ])).mean().as_matrix()
    return {"left": mean, "right": BODY_MIRROR @ mean @ local_mirror}


def fit_calibration(context, samples):
    """Fit raw world poses against a symmetric human neutral pose.

    ``samples`` maps chest, left/right_hand_back and left/right_arm to (N, 4, 4)
    arrays. Capture owns device validity and synchronization. The operator holds
    both palms down with fingers forward. Chest axes are X forward, Y left,
    Z up, with the origin at the chest Tracker. Upper-arm references retain raw
    Tracker orientation; this pose does not identify anatomical upper-arm twist.
    """
    profile = context["profile"]
    raw_chest = np.asarray(samples["chest"], dtype=float)
    if len(raw_chest) == 0:
        raise ValueError("Calibration requires a synchronized pose sample")

    chest_correction = np.eye(4)
    chest_correction[:3, :3] = np.asarray(profile["chest_rotation"], dtype=float)
    chest = raw_chest @ chest_correction
    chest_inverse_rotation = chest[:, :3, :3].transpose(0, 2, 1)
    data = {}
    for side in SIDES:
        raw_hand_back = np.asarray(samples[f"{side}_hand_back"], dtype=float)
        raw_upper = np.asarray(samples[f"{side}_arm"], dtype=float)
        hand_back_rotation = chest_inverse_rotation @ raw_hand_back[:, :3, :3]
        data[side] = {
            "hand_back_rotation": hand_back_rotation,
            "hand_back_position": np.einsum(
                "nij,nj->ni", chest_inverse_rotation,
                raw_hand_back[:, :3, 3] - chest[:, :3, 3],
            ),
            "upper_rotation": chest_inverse_rotation @ raw_upper[:, :3, :3],
            "palm_candidates": hand_back_rotation.transpose(0, 2, 1),
        }

    left, right = data["left"], data["right"]
    raw_to_palm_left = Rotation.from_matrix(np.concatenate([
        left["palm_candidates"],
        TRACKER_MIRROR @ right["palm_candidates"] @ BODY_MIRROR,
    ])).mean().as_matrix()
    raw_to_palm = {
        "left": raw_to_palm_left,
        "right": TRACKER_MIRROR @ raw_to_palm_left @ BODY_MIRROR,
    }

    for side in SIDES:
        current = data[side]
        offset = np.asarray(profile["hand_back_to_palm"][side]["tracker_offset_m"])
        current["palm_rotation"] = current["hand_back_rotation"] @ raw_to_palm[side]
        current["palm_position"] = (
            current["hand_back_position"] - current["palm_rotation"] @ offset
        )

    neutral_position_left = np.concatenate([
        left["palm_position"], right["palm_position"] @ BODY_MIRROR,
    ]).mean(axis=0)
    neutral_position = {
        "left": neutral_position_left,
        "right": BODY_MIRROR @ neutral_position_left,
    }
    neutral_palm = _paired_rotations(
        left["palm_rotation"], right["palm_rotation"], BODY_MIRROR,
    )
    # Upper Trackers can have independent mounting twists. Each neutral retains
    # its own measured raw orientation so a held neutral pose has zero delta.
    neutral_upper = {
        side: Rotation.from_matrix(data[side]["upper_rotation"]).mean().as_matrix()
        for side in SIDES
    }

    calibration = {
        "chest_rotation": deepcopy(profile["chest_rotation"]),
        # A single neutral capture cannot independently estimate mounting offset.
        "hand_back_to_palm": deepcopy(profile["hand_back_to_palm"]),
        "neutral_pose": {},
    }
    fit = {}
    for side in SIDES:
        current = data[side]
        for key, value in zip(("qx", "qy", "qz", "qw"), _quaternion(raw_to_palm[side])):
            calibration["hand_back_to_palm"][side][key] = value
        calibration["neutral_pose"][side] = {
            "position_m": neutral_position[side].tolist(),
            "quaternion_xyzw": _quaternion(neutral_palm[side]),
            "upper_arm_quaternion_xyzw": _quaternion(neutral_upper[side]),
        }

        position_error = np.linalg.norm(
            current["palm_position"] - neutral_position[side], axis=1,
        )
        palm_angle = np.rad2deg(Rotation.from_matrix(
            neutral_palm[side].T @ current["palm_rotation"],
        ).magnitude())
        upper_angle = np.rad2deg(Rotation.from_matrix(
            neutral_upper[side].T @ current["upper_rotation"],
        ).magnitude())
        fit[side] = {
            "mean_position_error_mm": float(position_error.mean() * 1000.0),
            "mean_palm_angle_deg": float(palm_angle.mean()),
            "mean_upper_angle_deg": float(upper_angle.mean()),
        }

    return {"calibration": calibration, "fit": fit}
