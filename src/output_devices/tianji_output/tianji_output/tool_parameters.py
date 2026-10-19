"""Tool geometry and cabinet/configured payload selection."""
import numpy as np

from ._internal.structure_data import DCSS


def tool_transform(config, side):
    """Return flange-to-control-frame geometry in millimetres."""
    value = (config.get("tool_transforms") or {}).get(side, np.eye(4).tolist())
    return np.asarray(value, dtype=float).tolist()


def resolve_dynamics(config, cabinet_inputs):
    """Select each arm's cabinet or configured ten-value payload array."""
    settings = config.get("tool_dynamics") or {"source": "controller"}
    source = settings.get("source", "controller")
    result = {}
    for index, side in enumerate(("left", "right")):
        if source == "controller":
            values = cabinet_inputs[index]["tool_dyn"]
        else:
            values = settings[side]
        result[side] = values
    return source, result


def apply_tool_parameters(robot, config, logger):
    """Apply selected payload values; tool geometry is handled by IK."""
    snapshot = robot.subscribe(DCSS())
    source, values = resolve_dynamics(config, snapshot["inputs"])
    logger.info(f"Tool payload source={source}: {values}")
    robot.clear_set()
    for arm, side in (("A", "left"), ("B", "right")):
        robot.set_tool(arm=arm, kineParams=[0, 0, 0, 0, 0, 0], dynamicParams=values[side])
    robot.send_cmd()
    return values
