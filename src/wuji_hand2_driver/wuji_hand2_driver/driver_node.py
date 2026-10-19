"""ROS 2 transport, with a dedicated fixed-rate SDK owner/command thread."""
from dataclasses import asdict
from pathlib import Path
import queue
import threading
import time

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .joint_mapping import FINGERS, JointLimits
from .safety import DriverConfig, HandSafety
from .sdk_backend import SdkBackend, Snapshot


class Hand2DriverNode(Node):
    def __init__(self):
        super().__init__("wuji_hand2_driver")
        defaults = {
            "serial_number": "", "expected_side": "", "expected_firmware": "2.5.1",
            "model_package": "wuji_hand2_2_description", "urdf_path": "",
            "read_only": False, "auto_enable": False, **asdict(DriverConfig()),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self._config = DriverConfig(**{name: float(param(name)) for name in asdict(DriverConfig())})
        self._read_only = bool(param("read_only"))
        self._auto_enable = bool(param("auto_enable"))
        if self._read_only and self._auto_enable:
            raise ValueError("read_only and auto_enable cannot both be true")
        self._serial_number = str(param("serial_number"))
        self._side = str(param("expected_side"))
        path = str(param("urdf_path"))
        if not path:
            path = str(Path(get_package_share_directory(str(param("model_package"))))
                       / "urdf" / f"{self._side}-ros.urdf")
        self._limits = JointLimits.from_urdf(path, self._side)
        self._backend = SdkBackend(self._serial_number, self._side,
                                   str(param("expected_firmware")), self._read_only)
        self._core = HandSafety(self._backend, self._limits, self._config, self._read_only)
        self._requests = queue.Queue(maxsize=32)
        self._command_lock = threading.Lock()
        self._pending_command = None
        self._report_lock = threading.Lock()
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
        self.create_timer(1.0 / self._config.publish_rate, self._publish_state)
        self.create_timer(1.0 / self._config.diagnostics_rate, self._publish_diagnostics)
        self._thread = threading.Thread(target=self._run, name="hand2-sdk-owner", daemon=True)
        self._thread.start()

    def _on_command(self, msg):
        if self._read_only:
            return
        received_at = time.monotonic()
        error = ""
        try:
            values = self._limits.validate_command(msg.position, msg.name)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if stamp <= 0 or age < -0.1 or age > self._config.command_timeout:
                raise ValueError("command header stamp is missing, stale, or in the future")
            if msg.velocity or msg.effort:
                raise ValueError("only position commands are accepted; velocity/effort must be empty")
        except (ValueError, TypeError) as exc:
            values = None
            error = str(exc)
        with self._command_lock:
            self._pending_command = (values, received_at, error)

    def _enqueue(self, action, response):
        if self._read_only:
            response.success, response.message = False, "read_only forbids all hardware writes"
            return response
        if not self._thread.is_alive():
            response.success = False
            response.message = "device owner is not running; fix the connection/identity error and restart the driver"
            return response
        with self._report_lock:
            state = self._report[0]
        if action == "enable" and state != "STOPPED":
            response.success, response.message = False, f"enable requires STOPPED, got {state}"
            return response
        if action == "clear_fault" and state != "FAULT":
            response.success, response.message = False, "clear_fault requires FAULT"
            return response
        try:
            self._requests.put_nowait(action)
            response.success = True
            response.message = f"{action} queued; completion is reported by lifecycle_state and diagnostics"
        except queue.Full:
            response.success, response.message = False, "driver request queue is full"
        return response

    def _on_enabled(self, request, response):
        return self._enqueue("enable" if request.data else "disable", response)

    def _on_emergency_stop(self, request, response):
        return self._enqueue("emergency_stop", response)

    def _on_clear_fault(self, request, response):
        return self._enqueue("clear_fault", response)

    def _save_report(self):
        report = (self._core.lifecycle, self._core.fault_reason, self._core.command_at,
                  self._backend.snapshot(), self._backend.firmware_version)
        with self._report_lock:
            self._report = report

    def _run(self):
        opened = False
        try:
            self._backend.connect()
            opened = True
            self._core.connected(time.monotonic())
            auto_pending = self._auto_enable
            period = 1.0 / self._config.command_rate
            deadline = time.monotonic()
            while not self._stop.is_set():
                while True:
                    try:
                        action = self._requests.get_nowait()
                    except queue.Empty:
                        break
                    # A manual stop/disable always cancels a pending one-shot
                    # auto-enable, even before the first telemetry frame.
                    auto_pending = False
                    if action == "emergency_stop":
                        self._core.fail("operator emergency_stop", emergency=True)
                    elif action == "clear_fault":
                        ok, message = self._core.clear_fault()
                        if not ok:
                            self.get_logger().warning(message)
                    else:
                        ok, message = self._core.set_enabled(action == "enable")
                        if not ok:
                            self.get_logger().warning(message)
                if auto_pending:
                    auto_pending = self._core.try_startup_enable()
                with self._command_lock:
                    command = self._pending_command
                    self._pending_command = None
                if command is not None:
                    values, received_at, error = command
                    if error and self._core.lifecycle in self._core.ARMED_STATES:
                        self._core.fail(f"invalid command: {error}")
                    elif not error:
                        self._core.command(values, (), received_at)
                self._core.tick()
                self._save_report()
                deadline += period
                remaining = deadline - time.monotonic()
                if remaining < 0:
                    # Skip missed ticks; never issue a burst of catch-up commands.
                    deadline = time.monotonic()
                else:
                    self._stop.wait(remaining)
        except Exception as exc:
            if opened:
                self._core.fail(f"SDK owner failed: {exc}")
            else:
                self._core.lifecycle = "FAULT"
                self._core.fault_reason = f"connection failed: {exc}"
            self._save_report()
            self.get_logger().error(self._core.fault_reason)
        finally:
            if opened:
                try:
                    self._core.close()
                except Exception as exc:
                    self.get_logger().error(f"driver shutdown: {exc}")
                if self._stop.is_set():
                    self._save_report()

    def _publish_state(self):
        with self._report_lock:
            lifecycle, reason, command_at, snapshot, firmware = self._report
        sample = snapshot.state
        # Never refresh ROS timestamps for an unchanged/stale SDK cache.
        if (sample is None or sample.received_at == self._last_published_sample
                or time.monotonic() - sample.received_at > self._config.feedback_timeout):
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._limits.names)
        msg.position = list(sample.positions)
        msg.velocity = list(sample.velocities)
        # Hand2 SDK effort is currently amperes (Kt placeholder), NOT Nm.
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
        status = DiagnosticStatus()
        status.name = f"{namespace}/hand"
        status.hardware_id = self._serial_number
        status.level = DiagnosticStatus.ERROR if lifecycle == "FAULT" else DiagnosticStatus.OK
        if lifecycle in ("DISCONNECTED", "ENABLING"):
            status.level = DiagnosticStatus.WARN
        status.message = reason or lifecycle
        joints = snapshot.diagnostics.joints if snapshot.diagnostics else ()
        values = {
            "serial_number": self._serial_number, "expected_side": self._side,
            "firmware_version": firmware, "lifecycle": lifecycle, "fault_reason": reason,
            "read_only": self._read_only, "feedback_age_sec": age(snapshot.state),
            "command_age_sec": -1.0 if command_at is None else max(0.0, now - command_at),
            "diagnostics_age_sec": age(snapshot.diagnostics),
            "enabled_joint_count": sum(j.ext_state == 2 for j in joints),
            "online_joint_count": len(joints),
        }
        status.values = [self._value(k, v) for k, v in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        for index, joint in enumerate(joints):
            entry = DiagnosticStatus()
            entry.name = f"{namespace}/{FINGERS[index // 4]}_S{index % 4 + 1}"
            entry.hardware_id = self._serial_number
            entry.level = DiagnosticStatus.ERROR if joint.error_code else DiagnosticStatus.OK
            entry.message = f"ext_state={joint.ext_state}"
            if age(snapshot.diagnostics) > self._config.diagnostics_timeout:
                entry.level = DiagnosticStatus.STALE
                entry.message = "stale SDK diagnostics"
            entry.values = [self._value(k, v) for k, v in asdict(joint).items()]
            msg.status.append(entry)
        self._diag_pub.publish(msg)
        self._lifecycle_pub.publish(String(data=lifecycle))
        if lifecycle != self._last_lifecycle:
            self.get_logger().info(f"Hand 2 lifecycle: {lifecycle}" + (f": {reason}" if reason else ""))
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
