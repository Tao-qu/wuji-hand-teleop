"""Send configured Home targets through the existing hand and arm controls."""
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from controller.common import ROS2LoggerAdapter, get_default_qos, load_yaml_config
from .launch_utils import resolve_config_path


class HomePositionNode(Node):
    """Keep Home targets active until the Home session is stopped."""

    def __init__(self):
        super().__init__("home_position")
        self.declare_parameter("mode", "both")
        self.declare_parameter("hand_config", "")
        self.declare_parameter("arm_config", "")
        mode = str(self.get_parameter("mode").value)
        if mode not in ("both", "arms", "hands"):
            raise ValueError("mode must be both, arms, or hands")
        self._use_arms = mode in ("both", "arms")
        self._hands = {}
        self._arm_controller = None
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
            from wujihand_output._internal.hand_interface import WujiHand

            hand_path = str(self.get_parameter("hand_config").value)
            hand_config = load_yaml_config(
                hand_path or resolve_config_path("wujihand_output", "hand2.yaml"))
            command_rate = float(hand_config.get("controller", {}).get("control_rate", 120.0))
            for side in ("left", "right"):
                config = hand_config[f"{side}_hand"]
                hand = WujiHand(config.get("name", f"{side}_hand"), side, self)
                self._hands[side] = (hand, config["home_joints"])

        self._arm_state_pubs = {}
        if self._use_arms:
            from tianji_output.configuration import load_arm_config

            self._arm_config = load_arm_config(
                str(self.get_parameter("arm_config").value) or None)
            state_rate = float(self._arm_config["controller"]["state_publish_rate"])
            for side in ("left", "right"):
                self._arm_state_pubs[side] = self.create_publisher(
                    JointState, f"/{side}_arm/joint_states", get_default_qos())
            self._timers.append(self.create_timer(1.0 / state_rate, self._publish_arm_feedback))

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
        try:
            from tianji_output import TianjiChestDriver

            self._arm_controller = TianjiChestDriver(
                config=self._arm_config,
                logger=ROS2LoggerAdapter(self.get_logger()))
            self._arm_controller.set_position_mode(velRatio=100, AccRatio=100)
            result = self._arm_controller.move_to_init(
                wait=True, timeout=15,
                duration=float(self._arm_config["controller"]["home_duration_sec"]))
            if result is False:
                self._set_error("Arm Home operation reported failure")
                return
            with self._state_lock:
                self._arm_done = True
        except Exception as exc:
            self._set_error(f"Arm Home failed: {exc}")

    def _publish_targets(self):
        if self._hands:
            results = [hand.set_joint_positions(target) for hand, target in self._hands.values()]
            with self._state_lock:
                self._hands_sent = all(results)
            if not all(results):
                self._set_error("Could not publish the configured hand Home targets")

    def _arm_message(self, side, positions):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = f"{side}_base_state"
        message.name = [f"{side}_joint_{index + 1}" for index in range(7)]
        # Match the existing Tianji ROS interface, which uses degrees.
        message.position = [float(value) for value in positions]
        return message

    def _publish_arm_feedback(self):
        controller = self._arm_controller
        if controller is None:
            return
        try:
            left, right = controller.get_current_joints()
            for side, positions in (("left", left), ("right", right)):
                if positions is not None:
                    self._arm_state_pubs[side].publish(self._arm_message(side, positions))
        except Exception as exc:
            self._set_error(f"Arm feedback failed: {exc}")

    def _publish_status(self):
        with self._state_lock:
            state = "ERROR" if self._error else "ACTIVE" if self._arm_done and self._hands_sent else "MOVING"
        self._state_pub.publish(String(data=state))
        if state != self._last_state:
            self.get_logger().info(f"Home state: {state}")
            if state == "ACTIVE":
                self.get_logger().info("Home targets active; Stop or Ctrl-C ends the session")
            self._last_state = state

    def close(self):
        for timer in self._timers:
            timer.cancel()
        try:
            if self._arm_controller is not None:
                self._arm_controller.disable_and_release()
        finally:
            for hand, _target in self._hands.values():
                hand.release()


def main(args=None):
    # Keep Python's SIGINT handler so it interrupts the existing arm move.
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
        while rclpy.ok():
            time.sleep(0.2)
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
