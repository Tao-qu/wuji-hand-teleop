"""Send configured Home targets through the existing hand and arm controls."""
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

from controller.common import load_yaml_config
from .launch_utils import resolve_config_path


class HomePositionNode(Node):
    """Send Home targets for one motion, then close the session."""

    def __init__(self):
        super().__init__("home_position")
        self.declare_parameter("mode", "both")
        mode = str(self.get_parameter("mode").value)
        if mode not in ("both", "arms", "hands"):
            raise ValueError("mode must be both, arms, or hands")
        self._use_arms = mode in ("both", "arms")
        self._hands = {}
        self._arm_home_client = None
        self._arm_finished = threading.Event()
        self._state_lock = threading.Lock()
        self._arm_done = not self._use_arms
        self._hands_sent = mode == "arms"
        self._error = ""
        self._last_state = None
        self._timers = []

        state_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(String, "/wuji_home/state", state_qos)
        command_rate = 120.0
        if mode in ("both", "hands"):
            from wujihand2_ros2._internal.hand_interface import WujiHand2

            hand_config = load_yaml_config(resolve_config_path("wujihand2_ros2", "wujihand2.yaml"))
            command_rate = float(hand_config.get("controller", {}).get("control_rate", 120.0))
            for side in ("left", "right"):
                config = hand_config[f"{side}_hand"]
                hand = WujiHand2(config.get("name", f"{side}_hand"), side, self)
                self._hands[side] = (hand, config["home_joints"])

        if self._use_arms:
            self._arm_home_client = self.create_client(Trigger, "/tianji_arm/home")

        self._timers.append(self.create_timer(1.0 / command_rate, self._publish_targets))
        self._timers.append(self.create_timer(0.2, self._publish_status))
        self._publish_status()

    def _set_error(self, message):
        with self._state_lock:
            changed = message != self._error
            self._error = message
        if changed:
            self.get_logger().error(message)

    def _home_arms(self):
        """Request the driver's existing Home motion and wait for its result."""
        while rclpy.ok() and not self._arm_home_client.wait_for_service(timeout_sec=0.2):
            pass
        if not rclpy.ok():
            return
        future = self._arm_home_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_arm_home_response)
        while rclpy.ok() and not self._arm_finished.wait(timeout=0.2):
            pass

    def _on_arm_home_response(self, future):
        """The response belongs to this request and reports completed motion."""
        try:
            response = future.result()
            if response.success:
                with self._state_lock:
                    self._arm_done = True
            else:
                self._set_error(f"Arm Home failed: {response.message}")
        except Exception as exc:
            self._set_error(f"Arm Home request failed: {exc}")
        finally:
            self._arm_finished.set()

    def _publish_targets(self):
        if self._hands:
            results = [hand.set_joint_positions(target) for hand, target in self._hands.values()]
            with self._state_lock:
                self._hands_sent = all(results)
            if not all(results):
                self._set_error("Could not publish the configured hand Home targets")

    def _publish_status(self):
        with self._state_lock:
            state = "ERROR" if self._error else "ACTIVE" if self._arm_done and self._hands_sent else "MOVING"
        self._state_pub.publish(String(data=state))
        if state != self._last_state:
            self.get_logger().info(f"Home state: {state}")
            self._last_state = state

    def close(self):
        for timer in self._timers:
            timer.cancel()
        for hand, _target in self._hands.values():
            hand.release()


def main(args=None):
    # Keep Python's SIGINT handler while waiting for the driver's Home result.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = None
    ros_thread = None
    try:
        node = HomePositionNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        def spin():
            try:
                executor.spin()
            except ExternalShutdownException:
                pass

        ros_thread = threading.Thread(target=spin, name="home-ros", daemon=True)
        ros_thread.start()
        if node._use_arms:
            node._home_arms()
        else:
            # Hand-only mode publishes Home targets for an eight-second motion.
            time.sleep(8.0)
        node._publish_status()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if ros_thread is not None:
            ros_thread.join()
        if node is not None:
            try:
                node.close()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
