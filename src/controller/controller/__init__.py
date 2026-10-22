"""Robot retargeting and controller nodes, with ROS loaded only when needed."""

from importlib import import_module

__all__ = [
    'ROS2LoggerAdapter',
    'TianjiArmControllerNode',
    'WujiHand2ControllerNode',
]


def __getattr__(name):
    modules = {
        'ROS2LoggerAdapter': '.common',
        'TianjiArmControllerNode': '.tianji_arm_node',
        'WujiHand2ControllerNode': '.wujihand2_node',
    }
    if name not in modules:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(modules[name], __name__), name)
    globals()[name] = value
    return value
