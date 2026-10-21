"""
Shared utilities — common classes and helpers used by controller nodes.

Includes:
- ROS2LoggerAdapter: ROS2 logger adapter
- Default QoS profile
- Config-loading helpers
"""
from __future__ import annotations

from pathlib import Path
import signal
from typing import Dict, Any

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.signals import SignalHandlerOptions


def run_ros_node(factory, args=None):
    """Stop callbacks and release hardware before closing the ROS context."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor()
    stopping = False
    initializing = True

    def interrupt(signum, frame):
        nonlocal stopping
        if not stopping:
            stopping = True
            # Let construction return its resources before cleanup takes over.
            if not initializing:
                raise KeyboardInterrupt

    previous = {sig: signal.signal(sig, interrupt)
                for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = factory()
        initializing = False
        if not stopping:
            executor.add_node(node)
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stopping = True
        try:
            executor.shutdown()
            if node is not None:
                try:
                    node.shutdown()
                finally:
                    node.destroy_node()
        finally:
            rclpy.try_shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)


class ROS2LoggerAdapter:
    """ROS2 logger adapter.

    Adapts a ROS2 logger to the standard logging interface (with stdlib
    %-format lazy formatting) so it can drive lower-level controller libraries.
    rclpy logger itself does not accept extra args; this adapter lazy-formats
    and then forwards to the ROS2 logger.

    Mirrors the equivalent implementation in
    the Python control libraries.
    """

    def __init__(self, ros_logger):
        self._logger = ros_logger

    def _format(self, msg, args):
        return msg % args if args else msg

    def info(self, msg, *args):
        self._logger.info(self._format(msg, args))

    def debug(self, msg, *args):
        self._logger.debug(self._format(msg, args))

    def warning(self, msg, *args):
        self._logger.warning(self._format(msg, args))

    def error(self, msg, *args):
        self._logger.error(self._format(msg, args))


def get_default_qos() -> QoSProfile:
    """Default QoS profile (for real-time control)."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


def load_yaml_config(config_path: str | Path) -> Dict[str, Any]:
    """Load a YAML config file.

    Args:
        config_path: path to the config file.

    Returns:
        Parsed config dict.

    Raises:
        FileNotFoundError: when the config file does not exist.
    """
    path = Path(config_path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}
