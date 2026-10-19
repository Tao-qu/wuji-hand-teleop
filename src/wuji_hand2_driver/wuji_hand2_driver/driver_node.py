"""ROS 2 transport for official Wuji Hand 2 commands and diagnostics."""
from dataclasses import asdict
import queue
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .joint_mapping import FINGERS, joint_names, joint_vector, nid_to_index
from .sdk_backend import SdkBackend, Snapshot


class Hand2DriverNode(Node):
    def __init__(self):
        super().__init__("wuji_hand2_driver")
        defaults = {
            "serial_number": "", "expected_side": "", "auto_enable": True,
            "command_rate": 200.0, "publish_rate": 200.0, "diagnostics_rate": 10.0,
            "kp": 3.0, "kd": 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self._serial_number = str(param("serial_number"))
        self._side = str(param("expected_side"))
        self._names = joint_names(self._side)
        self._auto_enable = bool(param("auto_enable"))
        self._command_rate = float(param("command_rate"))
        self._kp, self._kd = float(param("kp")), float(param("kd"))
        self._backend = SdkBackend(self._serial_number, self._side)
        self._requests = queue.Queue()
        self._command_lock = threading.Lock()
        self._latest_command = None
        self._report_lock = threading.Lock()
        self._lifecycle = "DISCONNECTED"
        self._operation_error = ""
        self._enabled = False
        self._command_at = None
        self._report = ("DISCONNECTED", "", None, Snapshot(None, None), "")
        self._stop = threading.Event()
        self._last_published_sample = None
        self._last_lifecycle = None
        self._command_sub = self.create_subscription(
            JointState, "joint_commands", self._on_command,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._state_pub = self.create_publisher(JointState, "joint_states", qos_profile_sensor_data)
        lifecycle_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._lifecycle_pub = self.create_publisher(String, "lifecycle_state", lifecycle_qos)
        self._diag_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.create_service(SetBool, "set_enabled", self._on_enabled)
        self.create_service(Trigger, "emergency_stop", self._on_emergency_stop)
        self.create_service(Trigger, "clear_fault", self._on_clear_fault)
        self.create_timer(1.0 / float(param("publish_rate")), self._publish_state)
        self.create_timer(1.0 / float(param("diagnostics_rate")), self._publish_diagnostics)
        self._thread = threading.Thread(target=self._run, name="hand2-sdk-owner", daemon=True)
        self._thread.start()

    def _on_command(self, msg):
        try:
            values = joint_vector(msg.position)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"Invalid joint command: {exc}")
            return
        with self._command_lock:
            self._latest_command = (values, time.monotonic())

    def _enqueue(self, action, response):
        if not self._thread.is_alive():
            response.success, response.message = False, "device connection is not running"
            return response
        self._requests.put(action)
        response.success = True
        response.message = f"{action} queued; SDK results are reported by diagnostics"
        return response

    def _on_enabled(self, request, response):
        return self._enqueue("enable" if request.data else "disable", response)

    def _on_emergency_stop(self, request, response):
        return self._enqueue("emergency_stop", response)

    def _on_clear_fault(self, request, response):
        return self._enqueue("clear_fault", response)

    def _save_report(self):
        report = (self._lifecycle, self._operation_error, self._command_at,
                  self._backend.snapshot(), self._backend.firmware_version)
        with self._report_lock:
            self._report = report

    def _action(self, action):
        if action == "enable":
            self._backend.enable()
            self._enabled = True
            self._lifecycle = "READY"
        elif action == "disable":
            self._backend.disable()
            self._enabled = False
            self._lifecycle = "STOPPED"
        elif action == "emergency_stop":
            self._backend.emergency_stop()
            self._enabled = False
            self._lifecycle = "STOPPED"
        elif action == "clear_fault":
            self._backend.clear_fault()
        self._operation_error = ""

    def _run(self):
        opened = False
        try:
            self._backend.connect()
            opened = True
            self._lifecycle = "STOPPED"
            self._backend.configure(self._kp, self._kd)
            if self._auto_enable:
                self._action("enable")
            period = 1.0 / self._command_rate
            deadline = time.monotonic()
            while not self._stop.is_set():
                while True:
                    try:
                        action = self._requests.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        self._action(action)
                    except Exception as exc:
                        self._operation_error = f"{action} failed: {exc}"
                        self.get_logger().error(self._operation_error)
                with self._command_lock:
                    command = self._latest_command
                if self._enabled and command is not None:
                    values, received_at = command
                    try:
                        self._backend.send(values)
                        self._command_at = received_at
                        self._lifecycle = "ACTIVE"
                        self._operation_error = ""
                    except Exception as exc:
                        self._operation_error = f"command send failed: {exc}"
                self._save_report()
                deadline += period
                remaining = deadline - time.monotonic()
                if remaining < 0:
                    deadline = time.monotonic()
                else:
                    self._stop.wait(remaining)
        except Exception as exc:
            self._operation_error = f"SDK connection failed: {exc}"
            self.get_logger().error(self._operation_error)
        finally:
            if opened:
                try:
                    self._backend.disable()
                except Exception as exc:
                    self.get_logger().error(f"driver shutdown disable: {exc}")
                try:
                    self._backend.close()
                except Exception as exc:
                    self.get_logger().error(f"driver shutdown: {exc}")
            self._enabled = False
            self._lifecycle = "DISCONNECTED"
            self._save_report()

    def _publish_state(self):
        with self._report_lock:
            lifecycle, reason, command_at, snapshot, firmware = self._report
        sample = snapshot.state
        if sample is None or sample.received_at == self._last_published_sample:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._names)
        msg.position = list(sample.positions)
        msg.velocity = list(sample.velocities)
        # SDK current is reported separately in amperes, not as JointState torque.
        msg.effort = []
        self._state_pub.publish(msg)
        self._last_published_sample = sample.received_at

    @staticmethod
    def _value(key, value):
        return KeyValue(key=key, value=str(value))

    def _publish_diagnostics(self):
        with self._report_lock:
            lifecycle, reason, command_at, snapshot, firmware = self._report
        now = time.monotonic()
        age = lambda sample: -1.0 if sample is None else max(0.0, now - sample.received_at)
        namespace = self.get_namespace().strip("/") or f"{self._side}_hand"
        joints = snapshot.diagnostics.joints if snapshot.diagnostics else ()
        status = DiagnosticStatus()
        status.name = f"{namespace}/hand"
        status.hardware_id = self._serial_number
        message = reason or snapshot.error
        status.level = DiagnosticStatus.ERROR if message or any(j.error_code for j in joints) else DiagnosticStatus.OK
        if lifecycle == "DISCONNECTED" and status.level == DiagnosticStatus.OK:
            status.level = DiagnosticStatus.WARN
        status.message = message or lifecycle
        values = {
            "serial_number": self._serial_number, "expected_side": self._side,
            "firmware_version": firmware, "lifecycle": lifecycle, "fault_reason": message,
            "feedback_age_sec": age(snapshot.state),
            "command_age_sec": -1.0 if command_at is None else max(0.0, now - command_at),
            "diagnostics_age_sec": age(snapshot.diagnostics),
            "enabled_joint_count": sum(j.ext_state == 2 for j in joints),
            "online_joint_count": len(joints),
        }
        status.values = [self._value(k, v) for k, v in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        for joint in joints:
            index = nid_to_index(joint.nid)
            entry = DiagnosticStatus()
            entry.name = f"{namespace}/{FINGERS[index // 4]}_S{index % 4 + 1}"
            entry.hardware_id = self._serial_number
            entry.level = DiagnosticStatus.ERROR if joint.error_code else DiagnosticStatus.OK
            entry.message = f"ext_state={joint.ext_state}"
            entry.values = [self._value(k, v) for k, v in asdict(joint).items()]
            msg.status.append(entry)
        self._diag_pub.publish(msg)
        self._lifecycle_pub.publish(String(data=lifecycle))
        if lifecycle != self._last_lifecycle:
            self.get_logger().info(f"Hand 2 lifecycle: {lifecycle}")
            self._last_lifecycle = lifecycle

    def close(self):
        self._stop.set()
        self._thread.join(timeout=10.0)
        if self._thread.is_alive():
            self.get_logger().error("SDK worker did not exit; hardware disable is unconfirmed")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Hand2DriverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
