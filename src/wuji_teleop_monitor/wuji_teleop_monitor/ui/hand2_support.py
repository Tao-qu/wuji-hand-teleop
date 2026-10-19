"""Hand 2 discovery and joint-display helpers."""

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
    """Arrange named feedback for the joint display."""
    if not names:
        return list(positions)
    values = dict(zip(names, positions))
    return [float(values.get(name, math.nan)) for name in joint_names(side)]


def scan_hand2(log, sdk=None):
    """Discover Hand 2 devices and read their metadata."""
    if sdk is None:
        import wuji_sdk as sdk
    manager = sdk.SdkManager.instance()
    results = []
    for discovered in manager.scan():
        if discovered.device_type != sdk.DeviceType.WujiHand2:
            continue
        item = {"sn": str(discovered.sn), "address": str(discovered.address),
                "transport": "Ethernet"}
        alias = "monitor_scan_" + uuid.uuid4().hex
        hand = None
        try:
            hand = manager.connect(sn=item["sn"], device_name=alias)
            side = str(hand.handedness().get()).lower()
            item.update(side=side)
            for key, read in (
                ("firmware", lambda: hand.device.get("firmware_version")),
                ("hardware", lambda: hand.hw_version().get()),
            ):
                try:
                    item[key] = str(read())
                except Exception as exc:
                    log(f"{item['sn']}: {key} unavailable: {exc}")
            log(f"{side.title()} Hand 2: {item['sn']} at {item['address']}")
        except Exception as exc:
            item["error"] = str(exc)
            log(f"Hand 2 details unavailable for {item['sn']}: {exc}")
        finally:
            if hand is not None:
                try:
                    manager.disconnect(alias)
                except Exception as exc:
                    log(f"Failed to release scan handle {alias}: {exc}")
        results.append(item)
    return results
