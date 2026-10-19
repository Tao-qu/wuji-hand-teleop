"""Read-only Hand 2 discovery and monitor-side health interpretation."""

from dataclasses import dataclass
import math
import uuid


FINGERS = ("thumb", "index", "middle", "ring", "pinky")
JOINT_LABELS = (
    ("CMC flex", "CMC abd", "MCP", "IP"),
    ("MCP flex", "MCP abd", "PIP", "DIP"),
)


def joint_names(side):
    """Return Hand 2 model names in firmware command order."""
    prefix = "l" if side == "left" else "r"
    names = [f"{prefix}_thumb_{suffix}" for suffix in ("cmc_flex", "cmc_abd", "mcp", "ip")]
    for finger in FINGERS[1:]:
        stem = finger if finger == "pinky" else f"{finger}_finger"
        names.extend(f"{prefix}_{stem}_{suffix}" for suffix in ("mcp_flex", "mcp_abd", "pip", "dip"))
    return names


def ordered_positions(side, names, positions):
    """Resolve named feedback without mistaking a partial frame for 20 joints."""
    if not names or len(names) != len(positions) or len(set(names)) != len(names):
        return [math.nan] * 20
    values = dict(zip(names, positions))
    return [float(values.get(name, math.nan)) for name in joint_names(side)]


def scan_hand2(log, sdk=None):
    """Discover Hand 2 devices and verify identity using GET operations only.

    No action, publisher, calibration, or parameter write is used. Only the
    temporary handles created by this scan are disconnected.
    """
    if sdk is None:
        import wuji_sdk as sdk
    manager = sdk.SdkManager.instance()
    results = []
    for discovered in manager.scan():
        if discovered.device_type != sdk.DeviceType.WujiHand2:
            continue
        item = {"sn": str(discovered.sn), "address": str(discovered.address),
                "transport": "Ethernet", "verified": False}
        alias = "monitor_scan_" + uuid.uuid4().hex
        hand = None
        try:
            hand = manager.connect(sn=item["sn"], device_name=alias)
            if not isinstance(hand, sdk.WujiHand2):
                raise ValueError("The connected device is not a Wuji Hand 2")
            if str(hand.serial_number) != item["sn"]:
                raise ValueError("Connected serial number does not match discovery")
            side = str(hand.handedness().get()).lower()
            if side not in ("left", "right"):
                raise ValueError(f"Unknown handedness: {side}")
            item.update(side=side, verified=True)
            for key, read in (
                ("firmware", lambda: hand.device.get("firmware_version")),
                ("hardware", lambda: hand.hw_version().get()),
            ):
                try:
                    item[key] = str(read())
                except Exception as exc:
                    log(f"{item['sn']}: {key} unavailable: {exc}")
            log(f"Verified {side} Hand 2: {item['sn']} at {item['address']}")
        except Exception as exc:
            item["error"] = str(exc)
            log(f"Hand 2 identity check failed for {item['sn']}: {exc}")
        finally:
            if hand is not None:
                try:
                    manager.disconnect(alias)
                except Exception as exc:
                    log(f"Failed to release scan handle {alias}: {exc}")
        results.append(item)
    return results


@dataclass
class HandHealth:
    lifecycle: str = ""
    feedback_at: float = 0.0
    diagnostics_at: float = 0.0
    commands_at: float = 0.0
    diagnostic_level: int = 0
    detail: str = ""
    complete_feedback: bool = False
    enabled_joint_count: int | None = None
    online_joint_count: int | None = None

    def assess(self, now, timeout=2.0):
        """Return display health, with live feedback required in every state."""
        if self.lifecycle in ("FAULT", "DISCONNECTED"):
            return "error", self.detail or self.lifecycle
        if not self.lifecycle:
            return "waiting", "Waiting for driver lifecycle"
        if self.diagnostic_level >= 2:
            return "error", self.detail or "Device diagnostic error"
        if not self.feedback_at or now - self.feedback_at > timeout:
            return "error", "Joint feedback missing or stale"
        if not self.diagnostics_at or now - self.diagnostics_at > timeout:
            return "error", "Diagnostics missing or stale"
        if not self.complete_feedback:
            return "error", "Incomplete joint feedback"
        if self.lifecycle == "ACTIVE":
            if not self.commands_at or now - self.commands_at > timeout:
                return "error", "Joint command stream stale"
            return "active", "ACTIVE"
        if self.lifecycle == "ENABLING":
            return "enabling", "ENABLING"
        if self.lifecycle == "STOPPED":
            if self.enabled_joint_count != 0 or self.online_joint_count != 20:
                return "waiting", "STOPPED (waiting for all 20 joints to confirm disabled)"
            return "ready", "STOPPED (disabled)"
        if self.lifecycle == "READY":
            return "ready", "READY (enabled; waiting for command)"
        if self.lifecycle == "READ_ONLY":
            return "ready", "READ_ONLY"
        return "error", f"Unknown driver lifecycle: {self.lifecycle}"
