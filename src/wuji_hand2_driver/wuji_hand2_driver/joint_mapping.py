"""Firmware-order joint names and SDK NID mapping for Wuji Hand 2."""

JOINT_COUNT = 20
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
_SUFFIXES = (
    "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip",
    "index_finger_mcp_flex", "index_finger_mcp_abd", "index_finger_pip", "index_finger_dip",
    "middle_finger_mcp_flex", "middle_finger_mcp_abd", "middle_finger_pip", "middle_finger_dip",
    "ring_finger_mcp_flex", "ring_finger_mcp_abd", "ring_finger_pip", "ring_finger_dip",
    "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip",
)


def joint_names(side):
    return tuple(f"{side[0]}_{suffix}" for suffix in _SUFFIXES)


def nid_to_index(nid):
    """Map the official 1–4, 6–9, 11–14, 16–19, 21–24 joint NIDs."""
    bus, offset = divmod(nid - 1, 5)
    return bus * 4 + offset


def ordered_entries(entries):
    """Place SDK entries in finger-major order using their wire NIDs."""
    return tuple(sorted(entries, key=lambda entry: nid_to_index(entry.nid)))


def joint_vector(values):
    """Convert the required twenty SDK joint values without imposing limits."""
    if len(values) != JOINT_COUNT:
        raise ValueError("joint positions must contain exactly 20 elements")
    return tuple(float(value) for value in values)
