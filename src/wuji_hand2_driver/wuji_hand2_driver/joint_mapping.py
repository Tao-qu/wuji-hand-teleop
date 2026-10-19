"""Explicit firmware order ↔ official hand2_2 URDF mapping (radians).

SDK stream NIDs contain a tactile slot between fingers. Never use nid - 1
as a command index, nor derive actuator order from XML traversal order.
"""
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

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
    if side not in ("left", "right"):
        raise ValueError("expected_side must be left or right")
    return tuple(f"{side[0]}_{suffix}" for suffix in _SUFFIXES)


def nid_to_index(nid):
    """Official bus*5+node layout: 1–4, 6–9, 11–14, 16–19, 21–24."""
    if not isinstance(nid, int) or isinstance(nid, bool):
        raise ValueError("NID must be an integer")
    bus, offset = divmod(nid - 1, 5)
    if not 0 <= bus < 5 or not 0 <= offset < 4:
        raise ValueError(f"not a joint NID: {nid}")
    return bus * 4 + offset


def ordered_entries(entries):
    if len(entries) != JOINT_COUNT:
        raise ValueError(f"incomplete joint frame: {len(entries)}/20")
    result = [None] * JOINT_COUNT
    for entry in entries:
        index = nid_to_index(entry.nid)
        if result[index] is not None:
            raise ValueError(f"duplicate joint NID: {entry.nid}")
        result[index] = entry
    if any(entry is None for entry in result):
        raise ValueError("joint frame is missing NIDs")
    return tuple(result)


def finite_vector(values, label="positions"):
    if len(values) != JOINT_COUNT:
        raise ValueError(f"{label} must contain exactly 20 elements")
    try:
        result = tuple(float(value) for value in values)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{label} contains non-numeric values") from exc
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains non-finite values")
    return result


@dataclass(frozen=True)
class JointLimits:
    names: tuple
    lower: tuple
    upper: tuple
    velocity: tuple

    @classmethod
    def from_urdf(cls, path, side):
        names = joint_names(side)
        root = ET.parse(Path(path)).getroot()
        movable = [j for j in root.findall("joint") if j.get("type") != "fixed"]
        by_name = {joint.get("name"): joint for joint in movable}
        if len(movable) != JOINT_COUNT or set(by_name) != set(names):
            raise ValueError("URDF must contain exactly the expected 20 Hand 2 joints")
        lower, upper, velocity = [], [], []
        for name in names:
            limit = by_name[name].find("limit")
            if limit is None:
                raise ValueError(f"missing URDF limit: {name}")
            lo, hi, vel = (float(limit.attrib[key]) for key in ("lower", "upper", "velocity"))
            if not all(math.isfinite(v) for v in (lo, hi, vel)) or lo >= hi or vel <= 0:
                raise ValueError(f"invalid URDF limit: {name}")
            lower.append(lo)
            upper.append(hi)
            velocity.append(vel)
        return cls(names, tuple(lower), tuple(upper), tuple(velocity))

    def validate_command(self, positions, names=()):
        values = finite_vector(positions)
        if names:
            if len(names) != JOINT_COUNT or len(set(names)) != JOINT_COUNT:
                raise ValueError("named command must contain 20 unique joint names")
            # Accept official full URDF names, or the exact unprefixed suffixes.
            canonical = tuple(n if n in self.names else f"{self.names[0][:2]}{n}" for n in names)
            if set(canonical) != set(self.names):
                raise ValueError("command joint names do not match this hand")
            by_name = dict(zip(canonical, values))
            values = tuple(by_name[name] for name in self.names)
        if any(q < lo - 1e-6 or q > hi + 1e-6
               for q, lo, hi in zip(values, self.lower, self.upper)):
            raise ValueError("command exceeds official URDF joint limits")
        return tuple(max(lo, min(hi, q)) for q, lo, hi in zip(values, self.lower, self.upper))
