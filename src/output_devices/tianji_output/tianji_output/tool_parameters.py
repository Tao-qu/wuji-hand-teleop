"""Explicit tool parameters; preserve cabinet payload until it is calibrated."""
import math
import time

import numpy as np

from ._internal.structure_data import DCSS
from .feedback_guard import DualArmFeedbackGuard


def tool_transform(config, side):
    """Return flange-to-control-frame transform (translation in millimetres)."""
    value = (config.get("tool_transforms") or {}).get(side, np.eye(4).tolist())
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1])
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1.0, atol=1e-6)):
        raise ValueError(f"tool_transforms.{side} must be a finite rigid 4x4 transform")
    return matrix.tolist()


def resolve_dynamics(config, cabinet_inputs):
    """Resolve [mass kg, COM mm x/y/z, inertia kg*mm^2 XX/XY/XZ/YY/YZ/ZZ]."""
    if not isinstance(config, dict):
        raise ValueError("tool configuration must be a mapping")
    settings = config.get("tool_dynamics") or {"source": "controller"}
    if not isinstance(settings, dict):
        raise ValueError("tool_dynamics must be a mapping")
    source = settings.get("source", "controller")
    if source not in ("controller", "configured"):
        raise ValueError("tool_dynamics.source must be controller or configured")
    result = {}
    for index, side in enumerate(("left", "right")):
        if source == "controller":
            if (not isinstance(cabinet_inputs, (list, tuple)) or len(cabinet_inputs) <= index
                    or not isinstance(cabinet_inputs[index], dict)):
                raise ValueError(f"Missing {side} tool dynamics from controller")
            values = cabinet_inputs[index].get("tool_dyn")
        else:
            values = settings.get(side)
        if not isinstance(values, (tuple, list)) or len(values) != 10:
            raise ValueError(f"Missing 10-value {side} tool dynamics from {source}")
        try:
            values = [float(v) for v in values]
        except (ValueError, TypeError) as exc:
            raise ValueError(f"Invalid {side} tool dynamics") from exc
        if not all(math.isfinite(v) for v in values) or values[0] < 0:
            raise ValueError(f"Invalid {side} tool dynamics")
        result[side] = values
    return source, result


def apply_tool_parameters(robot, config, logger):
    """Keep existing payload or apply measured values; IK owns tool geometry."""
    guard = DualArmFeedbackGuard()
    deadline = time.monotonic() + 2.0
    while True:
        snapshot = robot.subscribe(DCSS())
        try:
            guard.observe(snapshot)
            break
        except ValueError as exc:
            if time.monotonic() >= deadline:
                raise ValueError("cannot read payload without advancing dual-arm cabinet feedback") from exc
            time.sleep(0.01)
    source, values = resolve_dynamics(config, snapshot.get("inputs"))
    logger.info(f"Tool payload source={source}: {values}")
    robot.clear_set()
    for arm, side in (("A", "left"), ("B", "right")):
        robot.set_tool(arm=arm, kineParams=[0, 0, 0, 0, 0, 0], dynamicParams=values[side])
    robot.send_cmd()
    return values
