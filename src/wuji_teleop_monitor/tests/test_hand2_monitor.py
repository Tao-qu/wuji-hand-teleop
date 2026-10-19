"""Offline monitor contract tests; no real SDK manager or ROS node is created."""

import math
import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from wuji_teleop_monitor.ui.hand2_support import (
    HandHealth, joint_names, ordered_positions, scan_hand2,
)


def test_discovery_reads_identity_and_releases_only_its_handles():
    calls = []

    class Hand:
        serial_number = "WH2JA01TEST"
        device = SimpleNamespace(get=lambda path: calls.append(("get", path)) or "2.5.1")

        def handedness(self):
            return SimpleNamespace(get=lambda: "left")

        def hw_version(self):
            return SimpleNamespace(get=lambda: "0.2.0")

    manager = SimpleNamespace(
        scan=lambda: [SimpleNamespace(device_type="hand2", sn="WH2JA01TEST", address="192.168.1.110:7447"),
                      SimpleNamespace(device_type="glove", sn="ignore", address="other")],
        connect=lambda **kwargs: calls.append(("connect", kwargs)) or Hand(),
        disconnect=lambda alias: calls.append(("disconnect", alias)),
    )
    sdk = SimpleNamespace(SdkManager=SimpleNamespace(instance=lambda: manager),
                          DeviceType=SimpleNamespace(WujiHand2="hand2"), WujiHand2=Hand)
    found = scan_hand2(lambda _message: None, sdk=sdk)
    assert len(found) == 1
    assert found[0]["verified"] and found[0]["side"] == "left"
    assert found[0]["firmware"] == "2.5.1"
    assert calls[-1] == ("disconnect", calls[0][1]["device_name"])
    assert [kind for kind, _value in calls] == ["connect", "get", "disconnect"]


def test_feedback_uses_names_not_list_positions():
    names = joint_names("left")
    result = ordered_positions("left", names[::-1], list(range(20))[::-1])
    assert result == list(range(20))
    partial = ordered_positions("left", names[1:], list(range(1, 20)))
    assert math.isnan(partial[0]) and partial[1] == 1
    assert all(math.isnan(value) for value in ordered_positions("left", [], list(range(20))))


def test_health_detects_runtime_loss_and_does_not_require_commands_when_disabled():
    health = HandHealth(lifecycle="STOPPED", feedback_at=10, diagnostics_at=10,
                        complete_feedback=True, enabled_joint_count=0, online_joint_count=20)
    assert health.assess(11)[0] == "ready"
    health.lifecycle = "ACTIVE"
    health.commands_at = 10
    assert health.assess(11)[0] == "active"
    assert health.assess(13)[0] == "error"
    health.feedback_at = health.diagnostics_at = 13
    assert "command" in health.assess(13)[1]
    health.commands_at = 13
    health.diagnostic_level = 2
    assert health.assess(13)[0] == "error"
    health.diagnostic_level = 0
    health.lifecycle = "FAULT"
    assert health.assess(13)[0] == "error"


@pytest.fixture
def ui():
    pytest.importorskip("PyQt5")
    pytest.importorskip("ament_index_python")
    from PyQt5.QtWidgets import QApplication
    from wuji_teleop_monitor.ui.run_teleop import TeleopLauncherUI
    app = QApplication.instance() or QApplication([])
    window = TeleopLauncherUI(ros_node=None)
    yield window
    window._set_state("stopped")
    window.close()
    app.processEvents()


def test_ui_stays_disabled_without_explicit_enable_and_detects_running_loss(ui):
    import time
    ui._preset_combo.setCurrentText("Hands only (MANUS)")
    ui._process = SimpleNamespace(poll=lambda: None)
    ui._launch_started_at = time.monotonic() - 40
    ui._set_state("starting")
    now = time.monotonic()
    for side in ("left", "right"):
        ui._hand_health[side] = HandHealth(lifecycle="STOPPED", feedback_at=now,
                                         diagnostics_at=now, complete_feedback=True,
                                         enabled_joint_count=0, online_joint_count=20)
    ui._poll_process_alive()
    assert ui._state == "ready"
    assert not ui._pending_services
    for health in ui._hand_health.values():
        health.lifecycle = "ACTIVE"
        health.commands_at = now
    ui._poll_process_alive()
    assert ui._state == "running"
    ui._hand_health["left"].feedback_at = now - 5
    ui._poll_process_alive()
    assert ui._state == "error"
    assert "stale" in ui._chain_labels["left"].text()


def test_stopped_waits_for_complete_disabled_confirmation(ui):
    import time
    now = time.monotonic()
    health = HandHealth(lifecycle="STOPPED", feedback_at=now,
                        diagnostics_at=now, complete_feedback=True)
    ui._hand_health["left"] = health

    def diagnostics(enabled, online):
        values = [SimpleNamespace(key="enabled_joint_count", value=enabled),
                  SimpleNamespace(key="online_joint_count", value=online)]
        return SimpleNamespace(status=[SimpleNamespace(name="left_hand/hand", level=0,
                                                        message="STOPPED", values=values)])

    assert health.assess(now)[0] == "waiting"
    ui._on_hand_diagnostics("left", diagnostics("20", "20"))
    assert health.assess(now)[0] == "waiting"
    ui._on_hand_diagnostics("left", diagnostics("0", "19"))
    assert health.assess(now)[0] == "waiting"
    ui._on_hand_diagnostics("left", diagnostics("0", "20"))
    assert health.assess(now) == ("ready", "STOPPED (disabled)")
    ui._on_hand_diagnostics("left", diagnostics("invalid", "20"))
    assert health.assess(now)[0] == "waiting"


def test_joint_panel_has_current_units_and_valid_zero_pose(ui):
    panel = ui._joint_panel
    assert [panel._hand_joint_combo.itemText(i) for i in range(4)] == ["S1", "S2", "S3", "S4"]
    panel.update_hand_currents("left", [0.0] * 20)
    panel._hand_quantity_combo.setCurrentText("A")
    panel._refresh_display()
    assert panel._left_hand_labels[0].text() == "0.00"
    assert panel._left_hand_labels[0].property("_wuji_style") == panel.STYLE_NORMAL
    panel.update_hand_currents("left", [math.nan] * 20)
    panel._refresh_display()
    assert panel._left_hand_labels[0].text() == "--"


def test_unverified_or_wrong_side_identity_cannot_be_saved(ui, tmp_path):
    from wuji_teleop_monitor.ui.sn_dialog import ScanSNDialog, SaveToYamlDialog
    from PyQt5.QtWidgets import QPushButton
    path = tmp_path / "hand2.yaml"
    path.write_text('left_hand:\n  serial_number: old_left\nright_hand:\n  serial_number: old_right\n')
    dialog = ScanSNDialog(log_callback=lambda _message: None)
    # Do not show the dialog: showing it would start real network discovery.
    group = dialog._build_group('Hand 2', [dict(sn='new', verified=False)], path,
                                'hand', 'left_hand', 'right_hand')
    assert all(not button.isEnabled() for button in group.findChildren(QPushButton))
    group = dialog._build_group('Hand 2', [dict(sn='new', verified=True, side='left')], path,
                                'hand', 'left_hand', 'right_hand')
    assert [button.isEnabled() for button in group.findChildren(QPushButton)] == [True, False]
    duplicate = SaveToYamlDialog(ui, path, 'left_hand', 'hand', 'old_right', lambda _message: None)
    assert not duplicate._confirm_btn.isEnabled()
    assert 'already bound' in duplicate._diff_view.toPlainText()
    dialog._loading_timer.stop()
    dialog.deleteLater()
    duplicate.deleteLater()
