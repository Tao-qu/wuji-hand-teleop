#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Local MANUS / HTC teleoperation launcher and joint monitor."""

import fcntl
import math
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

# Shared UI components (sibling modules).
from .qt_setup import setup_qt_plugins
setup_qt_plugins()

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QPlainTextEdit, QMessageBox, QGroupBox,
    QFrame,
)
from PyQt5.QtCore import Qt, QTimer, QSize, pyqtSignal
from PyQt5.QtGui import QFont, QPixmap, QImage, QPainter
from PyQt5.QtSvg import QSvgRenderer

from .joint_panel import JointDisplayPanel
from .hand2_support import FINGERS, ordered_positions
from .sn_dialog import ScanSNDialog
from .theme import DARK_THEME_CSS, LOG_TEXTEDIT_CSS

# ROS2 imports (optional — joint preview degrades gracefully if absent).
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
    )
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int8, String
    from diagnostic_msgs.msg import DiagnosticArray
    from std_srvs.srv import SetBool
    _HAS_ROS2 = True
except ImportError:
    _HAS_ROS2 = False

# Lifecycle states published by tianji_arm_controller on
# /tianji_arm/lifecycle_state. Keep in sync with controller/tianji_arm_node.py.
_LC_INITIALIZING = 0
_LC_ENABLING = 1
_LC_READY = 2
_LC_DISABLED = 3
_LC_ENABLE_FAILED = 4
_LC_SDK_ERROR = 5
_LC_NAMES = {
    _LC_INITIALIZING: "initializing",
    _LC_ENABLING: "enabling",
    _LC_READY: "ready",
    _LC_DISABLED: "disabled",
    _LC_ENABLE_FAILED: "enable_failed",
    _LC_SDK_ERROR: "sdk_error",
}
# Single-instance lock.
_LOCK_FILE = "/tmp/wuji_teleop_monitor.lock"

# Each operation supports the same three device selections.
_BACKEND_MATCHES = (
    "ros2 launch wuji_teleop_bringup wuji_teleop.launch.py",
    "ros2 launch wuji_teleop_bringup wuji_home.launch.py",
)
LAUNCH_CONFIGS = {
    "Hands + Arms (MANUS + 5 Trackers)": ["wuji_teleop.launch.py", "enable_hand:=true", "enable_arm:=true"],
    "Hands only (MANUS)": ["wuji_teleop.launch.py", "enable_hand:=true", "enable_arm:=false"],
    "Arms only (5 Trackers)": ["wuji_teleop.launch.py", "enable_hand:=false", "enable_arm:=true"],
}
HOME_MODES = dict(zip(LAUNCH_CONFIGS, ("both", "hands", "arms")))


def _preset_waits_for_arm_lifecycle(preset):
    return "enable_arm:=true" in LAUNCH_CONFIGS.get(preset, [])


# Package logo (wuji.svg lives one level up from the ui/ subpackage).
_LOGO_PATH = Path(__file__).resolve().parent.parent / "wuji.svg"


@dataclass(eq=False)
class _LaunchSession:
    operation: str = "teleop"
    process: subprocess.Popen | None = None
    pgid: int | None = None
    cancelled: threading.Event = field(default_factory=threading.Event)
    spawn_done: threading.Event = field(default_factory=threading.Event)
    stop_failed: bool = False


class TeleopLauncherUI(QMainWindow):
    """One-click Wuji teleop launcher."""

    _MAX_LOG_LINES = 5000

    _launch_finished_signal = pyqtSignal(object)
    _stop_finished_signal = pyqtSignal(object, bool, bool)
    _arm_lifecycle_signal = pyqtSignal(int)
    _hand_lifecycle_signal = pyqtSignal(str, str)
    _home_state_signal = pyqtSignal(str)
    _service_result_signal = pyqtSignal(str, bool, str)

    def __init__(self, ros_node=None):
        super().__init__()
        self._session = None
        self._state = "stopped"
        self._start_time = None
        self._ros_node = ros_node

        self._launch_started_at = None
        self._output_lock = threading.Lock()
        self._pending_output = deque()
        self._stop_in_progress = False

        # Last lifecycle state we heard from tianji_arm_controller. None
        # means "no message yet" (the controller's TRANSIENT_LOCAL latch
        # hasn't reached us, or this preset doesn't have an arm controller).
        self._arm_lifecycle = None
        self._home_state = ""
        self._hand_seen = {}
        self._hand_commands_seen = {}
        self._hand_lifecycle = {side: '' for side in ('left', 'right')}
        self._enable_clients = {}
        self._waiting_for_arm_lifecycle = False

        self._launch_finished_signal.connect(self._on_launch_finished)
        self._stop_finished_signal.connect(self._on_stop_finished)
        self._arm_lifecycle_signal.connect(self._on_arm_lifecycle_main)
        self._hand_lifecycle_signal.connect(self._on_hand_lifecycle_main)
        self._home_state_signal.connect(self._on_home_state_main)
        self._service_result_signal.connect(self._on_service_result)

        self._init_ui()
        self._joint_panel.set_phase("stopped")
        self._preset_combo.currentTextChanged.connect(self._on_preset_changed)
        self._on_preset_changed(self._preset_combo.currentText())

        self._output_flush_timer = QTimer(self)
        self._output_flush_timer.setInterval(100)
        self._output_flush_timer.timeout.connect(self._flush_output)
        self._output_flush_timer.start()

        # Joint preview: subscribe to /{side}_{arm,hand}/joint_states.
        if self._ros_node and _HAS_ROS2:
            from .qos_utils import match_publisher_qos
            _lc_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            for side in ('left', 'right'):
                arm_topic = f'/{side}_arm/joint_states'
                self._ros_node.create_subscription(
                    JointState, arm_topic,
                    lambda msg, s=side: self._on_joint_state(s, msg),
                    match_publisher_qos(self._ros_node, arm_topic))
                hand_topic = f'/{side}_hand/joint_states'
                self._ros_node.create_subscription(
                    JointState, hand_topic,
                    lambda msg, s=side: self._on_hand_state(s, msg),
                    match_publisher_qos(self._ros_node, hand_topic))

                # Count commands from the active control chains.
                arm_cmd_topic = f'/{side}_arm/joint_commands'
                self._ros_node.create_subscription(
                    JointState, arm_cmd_topic,
                    lambda msg, s=side: self._joint_panel.record_hz(f'{s}_arm'),
                    match_publisher_qos(self._ros_node, arm_cmd_topic))
                hand_cmd_topic = f'/{side}_hand/joint_commands'
                self._ros_node.create_subscription(
                    JointState, hand_cmd_topic,
                    lambda msg, s=side: self._on_hand_command(s),
                    match_publisher_qos(self._ros_node, hand_cmd_topic))
                self._ros_node.create_subscription(
                    String, f'/{side}_hand/lifecycle_state',
                    lambda msg, s=side: self._hand_lifecycle_signal.emit(s, msg.data), _lc_qos)
                diag_topic = f'/{side}_hand/diagnostics'
                self._ros_node.create_subscription(
                    DiagnosticArray, diag_topic,
                    lambda msg, s=side: self._on_hand_diagnostics(s, msg),
                    match_publisher_qos(self._ros_node, diag_topic))
                self._enable_clients[side] = self._ros_node.create_client(
                    SetBool, f'/{side}_hand/set_enabled')

            # Latched lifecycle state from tianji_arm_controller. RELIABLE
            # + TRANSIENT_LOCAL matches the publisher so we receive the
            # last value even when subscribing before the controller comes
            # up. See _LC_* constants for the state machine.
            _lc_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self._ros_node.create_subscription(
                Int8, '/tianji_arm/lifecycle_state',
                self._on_arm_lifecycle, _lc_qos)
            self._ros_node.create_subscription(
                String, '/wuji_home/state',
                lambda msg: self._home_state_signal.emit(msg.data), _lc_qos)
            self._enable_clients['arms'] = self._ros_node.create_client(
                SetBool, '/tianji_arm_controller/set_enabled')

        # Uptime / liveness timers.
        self._uptime_timer = QTimer(self)
        self._uptime_timer.timeout.connect(self._update_uptime)
        self._uptime_timer.start(1000)

        self._liveness_timer = QTimer(self)
        self._liveness_timer.timeout.connect(self._poll_process_alive)
        self._liveness_timer.setInterval(500)

    @staticmethod
    def _render_logo(path: Path, target_height: int) -> QPixmap:
        if path.suffix.lower() == ".svg":
            renderer = QSvgRenderer(str(path))
            if not renderer.isValid():
                return QPixmap()
            src = renderer.defaultSize()
            if src.height() <= 0:
                return QPixmap()
            scale = target_height / src.height()
            size = QSize(max(1, int(round(src.width() * scale))), target_height)
            image = QImage(size, QImage.Format_ARGB32)
            image.fill(0)
            painter = QPainter(image)
            painter.setRenderHint(QPainter.Antialiasing, True)
            painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
            renderer.render(painter)
            painter.end()
            return QPixmap.fromImage(image)
        pixmap = QPixmap(str(path))
        if pixmap.isNull():
            return pixmap
        return pixmap.scaledToHeight(target_height, Qt.SmoothTransformation)

    def _init_ui(self):
        self.setWindowTitle("Wuji Teleop Monitor")
        # Portrait, ≈ golden ratio (760/620 ≈ 1.226). Fits the header +
        # Launch Configuration + 4-row joint readout + Process Output log
        # vertically; the Scan-SNs modal opens at 560×760 inside.
        self.setMinimumSize(620, 760)
        self.resize(720, 880)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(12)

        # Header: logo + status frame.
        header = QHBoxLayout()
        header.setSpacing(10)

        if _LOGO_PATH.exists():
            logo_label = QLabel()
            pixmap = self._render_logo(_LOGO_PATH, target_height=36)
            if pixmap is not None and not pixmap.isNull():
                logo_label.setPixmap(pixmap)
            logo_label.setFixedHeight(40)
            header.addWidget(logo_label)

        status_frame = QFrame()
        status_frame.setFixedHeight(40)
        status_frame.setStyleSheet(
            "QFrame { background-color: #333; border-radius: 6px; }")
        status_layout = QHBoxLayout(status_frame)
        status_layout.setContentsMargins(12, 4, 12, 4)

        self._status_label = QLabel("Teleop: stopped")
        self._status_label.setFont(QFont("Arial", 13, QFont.Bold))
        self._status_label.setStyleSheet("color: #aaa;")
        status_layout.addWidget(self._status_label)

        self._uptime_label = QLabel("")
        self._uptime_label.setFont(QFont("Courier", 11))
        self._uptime_label.setStyleSheet("color: #888;")
        self._uptime_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        status_layout.addWidget(self._uptime_label)
        header.addWidget(status_frame, 1)
        layout.addLayout(header)

        # Launch configuration.
        ctrl_group = QGroupBox("Launch Configuration")
        ctrl_layout = QVBoxLayout(ctrl_group)
        ctrl_layout.setSpacing(10)

        preset_row = QHBoxLayout()
        preset_lbl = QLabel("Preset:")
        preset_lbl.setFixedWidth(80)
        preset_lbl.setStyleSheet("color: #ccc; font-size: 12px;")
        preset_row.addWidget(preset_lbl)

        _COMBO_CSS = """
            QComboBox {
                background-color: #444; color: #ddd; border: 1px solid #666;
                border-radius: 4px; padding: 4px 10px; font-size: 12px;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView {
                background-color: #333; color: #ddd; selection-background-color: #555;
            }
        """
        self._preset_combo = QComboBox()
        self._preset_combo.addItems(list(LAUNCH_CONFIGS.keys()))
        self._preset_combo.setMinimumWidth(260)
        self._preset_combo.setStyleSheet(_COMBO_CSS)
        preset_row.addWidget(self._preset_combo)
        preset_row.addStretch()

        # Scan-SNs button: device grab conflicts with running teleop, so it
        # follows the same enable/disable gate as the preset combo.
        self._scan_sns_btn = QPushButton("Scan SNs")
        self._scan_sns_btn.setFixedHeight(28)
        self._scan_sns_btn.setStyleSheet("""
            QPushButton { background-color: #555; color: #ccc; border-radius: 4px;
                          padding: 2px 12px; font-size: 12px; }
            QPushButton:hover { background-color: #666; }
            QPushButton:disabled { background-color: #444; color: #777; }
        """)
        self._scan_sns_btn.clicked.connect(self._on_scan_sns)
        preset_row.addWidget(self._scan_sns_btn)
        ctrl_layout.addLayout(preset_row)

        # Start / Stop buttons.
        self._start_btn = QPushButton("Start Teleop")
        self._start_btn.setFixedHeight(55)
        self._start_btn.setFont(QFont("Arial", 14, QFont.Bold))
        self._start_btn.setStyleSheet("""
            QPushButton { background-color: #4CAF50; color: white; border-radius: 8px; }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        self._start_btn.clicked.connect(self._on_start)
        start_row = QHBoxLayout()
        start_row.addWidget(self._start_btn)
        self._home_btn = QPushButton("Return Home")
        self._home_btn.setFixedHeight(55)
        self._home_btn.setFont(QFont("Arial", 14, QFont.Bold))
        self._home_btn.setStyleSheet("""
            QPushButton { background-color: #1565C0; color: white; border-radius: 8px; }
            QPushButton:hover { background-color: #1976D2; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        self._home_btn.setToolTip("Return the selected hands and/or arms to their configured Home positions")
        self._home_btn.clicked.connect(self._on_home)
        start_row.addWidget(self._home_btn)
        ctrl_layout.addLayout(start_row)

        self._stop_btn = QPushButton("Stop Teleop")
        self._stop_btn.setFixedHeight(55)
        self._stop_btn.setFont(QFont("Arial", 14, QFont.Bold))
        self._stop_btn.setStyleSheet("""
            QPushButton { background-color: #C62828; color: white; border-radius: 8px; }
            QPushButton:hover { background-color: #B71C1C; }
            QPushButton:disabled { background-color: #555; color: #888; }
        """)
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._on_stop)
        ctrl_layout.addWidget(self._stop_btn)

        self._chain_labels = {}
        self._enable_buttons = {}
        self._disable_buttons = {}
        for chain, title in (("left", "Left Hand 2"), ("right", "Right Hand 2"), ("arms", "Arms")):
            row = QHBoxLayout()
            label = QLabel(f"{title}: stopped")
            label.setWordWrap(True)
            row.addWidget(label, 1)
            self._chain_labels[chain] = label
            enable = QPushButton(f"Enable {title}")
            disable = QPushButton(f"Disable {title}")
            for button in (enable, disable):
                button.setStyleSheet(
                    'QPushButton { background: #455A64; color: #eee; padding: 5px 8px; border-radius: 4px; }'
                    'QPushButton:hover { background: #546E7A; }'
                    'QPushButton:disabled { background: #383838; color: #888; }')
            enable.setEnabled(False)
            disable.setEnabled(False)
            enable.clicked.connect(lambda _checked=False, c=chain: self._request_enabled(c, True))
            disable.clicked.connect(lambda _checked=False, c=chain: self._request_enabled(c, False))
            row.addWidget(enable)
            row.addWidget(disable)
            self._enable_buttons[chain] = enable
            self._disable_buttons[chain] = disable
            ctrl_layout.addLayout(row)
        layout.addWidget(ctrl_group)

        # Joint preview (shared component).
        self._joint_panel = JointDisplayPanel()
        layout.addWidget(self._joint_panel)

        # Process output.
        log_group = QGroupBox("Process output")
        log_layout = QVBoxLayout(log_group)

        self._output_text = QPlainTextEdit()
        self._output_text.setReadOnly(True)
        self._output_text.setFont(QFont("Courier New", 9))
        self._output_text.setStyleSheet(LOG_TEXTEDIT_CSS)
        self._output_text.setPlaceholderText("Launch output appears here...")
        self._output_text.document().setMaximumBlockCount(self._MAX_LOG_LINES)
        log_layout.addWidget(self._output_text)

        clear_btn = QPushButton("Clear log")
        clear_btn.setFixedHeight(30)
        clear_btn.setStyleSheet("""
            QPushButton { background-color: #555; color: #ccc; border-radius: 4px; }
            QPushButton:hover { background-color: #666; }
        """)
        clear_btn.clicked.connect(self._clear_output)
        log_layout.addWidget(clear_btn)
        layout.addWidget(log_group, 1)

        self.setStyleSheet(DARK_THEME_CSS)

    def _on_preset_changed(self, preset):
        args = LAUNCH_CONFIGS[preset]
        self._joint_panel.set_mode(hands="enable_hand:=true" in args,
                                   arms="enable_arm:=true" in args)
        self._scan_sns_btn.setEnabled("enable_hand:=true" in args)
        for chain in self._chain_labels:
            visible = ("enable_arm:=true" if chain == "arms" else "enable_hand:=true") in args
            self._chain_labels[chain].setVisible(visible)
            self._enable_buttons[chain].setVisible(visible)
            self._disable_buttons[chain].setVisible(visible)

    # -------------------- Scan SNs --------------------

    def _on_scan_sns(self):
        dlg = ScanSNDialog(parent=self, log_callback=self._emit_output)
        dlg.exec_()

    # -------------------- ROS2 callbacks --------------------

    def _on_joint_state(self, side: str, msg):
        positions = list(msg.position[:7]) if len(msg.position) >= 7 else list(msg.position)
        self._joint_panel.update_joints(side, positions)

    def _on_hand_state(self, side, msg):
        now = time.monotonic()
        positions = ordered_positions(side, list(msg.name), list(msg.position))
        self._hand_seen[side] = now
        self._joint_panel.update_hand_joints(side, positions)

    def _on_hand_command(self, side):
        now = time.monotonic()
        self._hand_commands_seen[side] = now
        self._joint_panel.record_hz(f'{side}_hand')

    def _on_hand_diagnostics(self, side, msg):
        """Display motor current from the driver diagnostics."""
        statuses = [status for status in msg.status if status.name.startswith(f'{side}_hand/')]
        if not statuses:
            return
        currents = [math.nan] * 20
        for status in statuses:
            values = {value.key: value.value for value in status.values}
            label = status.name.rsplit('/', 1)[-1]
            for finger_idx, finger in enumerate(FINGERS):
                for joint in range(4):
                    if label == f'{finger}_S{joint + 1}':
                        try:
                            currents[finger_idx * 4 + joint] = float(values['current_a'])
                        except (KeyError, TypeError, ValueError):
                            pass
        self._joint_panel.update_hand_currents(side, currents)

    def _on_hand_lifecycle_main(self, side, lifecycle):
        lifecycle = lifecycle.strip().upper()
        previous = self._hand_lifecycle[side]
        self._hand_lifecycle[side] = lifecycle
        if previous != lifecycle:
            self._emit_output(f'>>> {side}_hand/lifecycle_state: {lifecycle}')
        self._poll_process_alive()

    def _request_enabled(self, chain, enabled):
        """Forward the requested motor state to the driver."""
        client = self._enable_clients.get(chain)
        if client is None or not client.service_is_ready():
            self._emit_output(f'>>> {chain}: set_enabled service is unavailable')
            return
        request = SetBool.Request()
        request.data = enabled
        try:
            future = client.call_async(request)
        except Exception as exc:
            self._emit_output(f'>>> {chain}: service request failed: {exc}')
            return
        self._emit_output(f'>>> {chain}: requested {"enable" if enabled else "disable"}')

        def completed(result):
            try:
                response = result.result()
                self._service_result_signal.emit(chain, response.success, response.message)
            except Exception as exc:
                self._service_result_signal.emit(chain, False, str(exc))
        future.add_done_callback(completed)

    def _on_service_result(self, chain, success, message):
        self._emit_output(f'>>> {chain}: {"accepted" if success else "failed"}: {message}')
        self._poll_process_alive()

    def _on_arm_lifecycle(self, msg):
        # ROS2 spin thread — bounce to GUI thread.
        self._arm_lifecycle_signal.emit(int(msg.data))

    def _on_arm_lifecycle_main(self, lc: int):
        """Update arm status without completing a pending process stop."""
        prev = self._arm_lifecycle
        self._arm_lifecycle = lc
        if prev != lc:
            self._emit_output(
                f">>> tianji_arm/lifecycle_state: "
                f"{_LC_NAMES.get(lc, str(lc))}"
            )
        if (not self._is_home_session()
                and _preset_waits_for_arm_lifecycle(self._preset_combo.currentText())
                and self._state in ("starting", "ready", "running", "error")):
            if lc in (_LC_ENABLE_FAILED, _LC_SDK_ERROR):
                self._set_state("error")
            else:
                self._poll_process_alive()

    # -------------------- State management --------------------

    def _is_home_session(self):
        return self._session is not None and self._session.operation == "home"

    def _on_home_state_main(self, state):
        if self._is_home_session():
            self._home_state = state.strip().upper()
            self._poll_process_alive()

    def _set_state(self, state: str):
        self._state = state
        #                       text,                   color,    start_en, config_en
        _states = {
            "stopped":   ("Teleop: stopped",        "#aaa",    True,  True),
            "starting":  ("Teleop: starting...",    "#F57C00", False, False),
            "ready":     ("Teleop: ready — check device enable and input status", "#4FC3F7", False, False),
            "running":   ("Teleop: running",        "#4CAF50", False, False),
            "error": ("Teleop: device or stream fault — inspect status and logs", "#C62828", False, False),
            "stopping":  ("Teleop: stopping...",    "#F57C00", False, False),
        }
        text, color, start_en, config_en = _states.get(state, _states["stopped"])
        if self._is_home_session():
            text = {
                "starting": "Home: moving to configured positions...",
                "ready": "Home: targets active — check device enable status",
                "running": "Home: configured targets active",
                "error": "Home: operation or device error — inspect logs",
                "stopping": "Home: stopping...",
            }.get(state, text)
        self._status_label.setText(text)
        self._status_label.setStyleSheet(f"color: {color};")
        self._start_btn.setEnabled(start_en)
        self._home_btn.setEnabled(start_en)
        self._stop_btn.setText("Stop Home" if self._is_home_session() else "Stop Teleop")
        self._stop_btn.setEnabled(state in ("starting", "ready", "running", "error"))
        self._preset_combo.setEnabled(config_en)
        self._scan_sns_btn.setEnabled(config_en and "enable_hand:=true" in LAUNCH_CONFIGS[self._preset_combo.currentText()])
        self._joint_panel.set_phase(state)
        if state in ('starting', 'stopping'):
            for chain in self._enable_buttons:
                self._enable_buttons[chain].setEnabled(False)
                self._disable_buttons[chain].setEnabled(False)

        if state == "stopped":
            self._uptime_label.setText("")
            self._start_time = None
            self._liveness_timer.stop()
            self._waiting_for_arm_lifecycle = False
            self._arm_lifecycle = None
            for chain, label in self._chain_labels.items():
                label.setText(f'{chain.title()}: stopped')
                self._enable_buttons[chain].setEnabled(False)
                self._disable_buttons[chain].setEnabled(False)
        elif state == "running":
            if self._start_time is None:
                self._start_time = self._launch_started_at or time.monotonic()
        elif state == "starting":
            self._liveness_timer.start()

        if state == "starting" and self._waiting_for_arm_lifecycle:
            self._status_label.setText("Teleop: starting (arm init...)")

    # -------------------- Backend process discovery --------------------

    def _find_backend_processes(self):
        try:
            output = subprocess.check_output(
                ["ps", "-eo", "pid=,pgid=,args="],
                text=True,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            return []

        processes = []
        for line in output.splitlines():
            if not any(pattern in line for pattern in _BACKEND_MATCHES):
                continue
            parts = line.strip().split(None, 2)
            if len(parts) < 3:
                continue
            try:
                pid = int(parts[0])
                pgid = int(parts[1])
            except ValueError:
                continue
            processes.append({"pid": pid, "pgid": pgid, "args": parts[2]})
        return processes

    def _poll_process_alive(self):
        """Continuously check every selected driver, including after startup."""
        if self._state not in ('starting', 'ready', 'running', 'error'):
            return
        session = self._session
        if session is None or session.stop_failed or not session.spawn_done.is_set():
            return
        if session.process is None or session.process.poll() is not None:
            self._on_launch_finished(session)
            return
        args = LAUNCH_CONFIGS[self._preset_combo.currentText()]
        outcomes = []
        home = self._is_home_session()
        home_outcome = {"MOVING": "waiting", "ACTIVE": "active", "ERROR": "error"}.get(self._home_state, "waiting")
        if home:
            outcomes.append(home_outcome)
        if "enable_arm:=true" in args:
            lc = self._arm_lifecycle
            if home:
                outcome = home_outcome
            elif lc == _LC_READY:
                outcome = 'active'
            elif lc == _LC_DISABLED:
                outcome = 'ready'
            elif lc in (_LC_ENABLE_FAILED, _LC_SDK_ERROR):
                outcome = 'error'
            else:
                outcome = 'waiting'
            outcomes.append(outcome)
            arm_status = (self._home_state or "waiting for Home") if home else _LC_NAMES.get(lc, "waiting for driver")
            self._chain_labels['arms'].setText(f'Arms: {arm_status}')
            self._update_enable_buttons('arms')
        if "enable_hand:=true" in args:
            for side in ('left', 'right'):
                lifecycle = self._hand_lifecycle[side]
                outcome = {'ACTIVE': 'active', 'READY': 'ready', 'STOPPED': 'ready',
                           'FAULT': 'error', 'DISCONNECTED': 'error'}.get(lifecycle, 'waiting')
                self._chain_labels[side].setText(f'{side.title()} Hand 2: {lifecycle or "waiting for driver"}')
                color = '#C62828' if outcome == 'error' else '#4CAF50' if outcome == 'active' else '#FFB74D'
                self._chain_labels[side].setStyleSheet(f'color: {color};')
                self._update_enable_buttons(side)
                outcomes.append(outcome)
        if 'error' in outcomes:
            state = 'error'
        elif 'waiting' in outcomes:
            state = 'starting'
        elif outcomes and all(outcome == 'active' for outcome in outcomes):
            state = 'running'
        else:
            state = 'ready'
        if state != self._state:
            self._set_state(state)

    def _update_enable_buttons(self, chain):
        client = self._enable_clients.get(chain)
        available = client is not None and client.service_is_ready()
        self._enable_buttons[chain].setEnabled(available)
        self._disable_buttons[chain].setEnabled(available)

    # -------------------- Launch --------------------

    def _build_launch_cmd(self, preset: str, operation: str = "teleop") -> list:
        args = (["wuji_home.launch.py", f"mode:={HOME_MODES[preset]}"]
                if operation == "home" else LAUNCH_CONFIGS[preset])
        # `ros2 launch <package> <launch_file> <args...>`
        # bash -c lets us pick up overlay setup files if the user happens
        # to have one sourced.
        ros2_args = " ".join(args)
        launch_cmd = (
            f"exec ros2 launch wuji_teleop_bringup {ros2_args}"
        )
        return ["bash", "-c", launch_cmd]

    def _on_start(self):
        self._start_operation("teleop")

    def _on_home(self):
        self._start_operation("home")

    def _start_operation(self, operation):
        if self._state != "stopped":
            return
        existing = self._find_backend_processes()
        if existing:
            QMessageBox.warning(
                self, "Session already running",
                "An existing Teleop or Home launch was detected on this machine.\n\n"
                "Stop the other instance first, then try again.",
            )
            return

        preset = self._preset_combo.currentText()
        if operation == "teleop" and _preset_waits_for_arm_lifecycle(preset):
            reply = QMessageBox.warning(
                self, 'Start arms',
                'The arms will move to their initial pose and enter impedance mode.\n'
                'Keep the workspace clear and the e-stop within reach.',
                QMessageBox.Ok | QMessageBox.Cancel, QMessageBox.Cancel)
            if reply != QMessageBox.Ok:
                return
        self._hand_seen.clear()
        self._hand_commands_seen.clear()
        self._hand_lifecycle = {side: '' for side in ('left', 'right')}
        self._waiting_for_arm_lifecycle = operation == "teleop" and _preset_waits_for_arm_lifecycle(preset)
        self._arm_lifecycle = None
        self._home_state = ""
        cmd = self._build_launch_cmd(preset, operation)

        session = _LaunchSession(operation=operation)
        self._session = session
        self._set_state("starting")
        self._clear_output()
        self._launch_started_at = time.monotonic()
        self._emit_output(f">>> Preset: {preset}")
        self._emit_output(">>> " + cmd[-1])

        def _run():
            proc = None
            try:
                try:
                    if session.cancelled.is_set():
                        return
                    proc = subprocess.Popen(
                        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        text=True, bufsize=1, start_new_session=True)
                    session.process = proc
                    session.pgid = proc.pid
                finally:
                    # Stop can now either cancel the pending start or own its group.
                    session.spawn_done.set()
                self._emit_output(f">>> Launch command issued; waiting for {operation} to initialize...")
                for line in iter(proc.stdout.readline, ''):
                    if not line:
                        break
                    self._emit_output(line.rstrip())
                retcode = proc.wait()
                self._emit_output(f"\n>>> Process exited (code: {retcode})")
            except Exception as exc:
                self._emit_output(f"\n>>> Launch error: {exc}")
            finally:
                if proc is not None and proc.stdout is not None:
                    proc.stdout.close()
                self._launch_finished_signal.emit(session)

        threading.Thread(target=_run, daemon=True, name=f"{operation}-launch").start()

    def _on_launch_finished(self, session):
        if session is not self._session or self._stop_in_progress or session.stop_failed:
            return
        try:
            alive = self._session_alive(session)
        except Exception as exc:
            self._emit_output(f">>> Could not inspect session processes: {exc}")
            alive = True
        if alive:
            self._set_state("error")
            self._status_label.setText(f"{session.operation.title()}: launcher exited; stop remaining processes")
        else:
            self._session = None
            self._set_state("stopped")

    def _on_stop(self):
        if self._state in ("stopped", "stopping"):
            return
        self._start_async_stop(close_after=False, reason="Stopping current session...")

    def _start_async_stop(self, close_after: bool, reason: str):
        if self._stop_in_progress:
            return
        session = self._session
        if session is None:
            self._set_state("stopped")
            if close_after:
                self.close()
            return
        session.cancelled.set()
        session.stop_failed = False
        self._stop_in_progress = True
        self._set_state("stopping")
        self._emit_output(f"\n>>> {reason}")
        self._emit_output(">>> Waiting for the session to shut down (up to 30s)...")

        def _stop():
            try:
                session.spawn_done.wait()
                ok = self._graceful_stop_sync(session)
            except Exception as exc:
                self._emit_output(f">>> Stop failed: {exc}")
                ok = False
            self._stop_finished_signal.emit(session, ok, close_after)

        threading.Thread(target=_stop, daemon=True, name="teleop-stop").start()

    def _on_stop_finished(self, session, ok: bool, close_after: bool):
        if session is not self._session:
            return
        self._stop_in_progress = False
        if ok:
            self._session = None
            self._emit_output(f">>> {session.operation.title()} stopped")
            self._set_state("stopped")
            if close_after:
                self.close()
            return

        session.stop_failed = True
        self._emit_output(">>> Stop incomplete: retry Stop")
        self._set_state("error")
        self._status_label.setText(f"{session.operation.title()}: stop incomplete — retry Stop")

    def _emit_output(self, text: str):
        with self._output_lock:
            self._pending_output.append(text)

    def _clear_output(self):
        with self._output_lock:
            self._pending_output.clear()
        self._output_text.clear()

    def _flush_output(self):
        with self._output_lock:
            if not self._pending_output:
                return
            text = "\n".join(self._pending_output)
            self._pending_output.clear()
        self._output_text.appendPlainText(text)
        scrollbar = self._output_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _update_uptime(self):
        if self._start_time is None:
            return
        elapsed = int(time.monotonic() - self._start_time)
        h, m, s = elapsed // 3600, (elapsed % 3600) // 60, elapsed % 60
        self._uptime_label.setText(f"Uptime: {h:02d}:{m:02d}:{s:02d}")

    @staticmethod
    def _session_alive(session) -> bool:
        if session.pgid is None:
            return False
        # Reap our launcher; zombies do not have running code to stop.
        if session.process is not None:
            session.process.poll()
        output = subprocess.check_output(
            ["ps", "-eo", "pgid=,stat="], text=True,
            stderr=subprocess.DEVNULL, timeout=2.0)
        for line in output.splitlines():
            fields = line.split()
            if len(fields) == 2 and int(fields[0]) == session.pgid:
                if not fields[1].startswith(('Z', 'X')):
                    return True
        return False

    def _graceful_stop_sync(self, session) -> bool:
        if not self._session_alive(session):
            return True
        pgid = session.pgid
        self._emit_output(f">>> Sending SIGINT to session process group: {pgid}")
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            return True

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if not self._session_alive(session):
                return True
            time.sleep(0.2)

        self._emit_output(f">>> Timed out (30s); sending SIGKILL to process group: {pgid}")
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            return True

        kill_deadline = time.monotonic() + 5.0
        while time.monotonic() < kill_deadline:
            if not self._session_alive(session):
                return True
            time.sleep(0.2)
        return False

    def closeEvent(self, event):
        if self._stop_in_progress or self._state == "stopping":
            event.ignore()
            return

        if self._state != "stopped":
            reply = QMessageBox.question(
                self, "Confirm exit",
                "The current session is still running.\n\n"
                "Closing the window will stop it first.\n\n"
                "Stop and exit?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.No:
                event.ignore()
                return
            self._start_async_stop(
                close_after=True,
                reason="Closing window: stopping the current session...")
            event.ignore()
            return

        # Stop timers before widgets are destroyed.
        self._uptime_timer.stop()
        self._liveness_timer.stop()
        self._output_flush_timer.stop()
        self._flush_output()
        event.accept()


def _acquire_lock():
    try:
        fd = open(_LOCK_FILE, 'w')
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fd.write(str(os.getpid()))
        fd.flush()
        return fd
    except (IOError, OSError):
        print("Error: Teleop Monitor is already running (another instance holds the lock file).")
        try:
            app = QApplication(sys.argv)
            QMessageBox.critical(
                None, "Wuji Teleop Monitor",
                "Teleop Monitor is already running!\n\n"
                "Only one instance is allowed at a time.\n"
                "Close the existing window first.")
        except Exception:
            pass
        sys.exit(1)


def main():
    lock_fd = _acquire_lock()

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    ros_node = None
    if _HAS_ROS2:
        try:
            rclpy.init()
            ros_node = Node('wuji_teleop_monitor')

            def _spin():
                try:
                    rclpy.spin(ros_node)
                except Exception:
                    pass

            threading.Thread(target=_spin, daemon=True, name="ros2-spin").start()
        except Exception as e:
            print(f"Warning: ROS2 init failed ({e}); joint preview disabled")
            ros_node = None

    window = TeleopLauncherUI(ros_node=ros_node)
    window.show()
    exit_code = app.exec_()

    # Graceful ROS2 shutdown: shutdown first so spin() returns, then destroy_node.
    if ros_node:
        try:
            rclpy.shutdown()
        except Exception:
            pass
        try:
            ros_node.destroy_node()
        except Exception:
            pass

    lock_fd.close()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
