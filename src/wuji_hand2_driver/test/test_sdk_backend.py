from types import SimpleNamespace
import threading

import pytest

from wuji_hand2_driver.sdk_backend import DeviceOwner, SdkBackend


class Resource:
    def __init__(self, hand, name, value=None):
        self.hand, self.name, self.value = hand, name, value

    def get(self):
        self.hand.calls.append(("get", self.name))
        return self.value

    def set(self, value): self.hand.calls.append(("set", self.name, value))
    def close(self): self.hand.calls.append(("close", self.name))

    def subscribe_with_callback(self, callback, on_error):
        self.hand.calls.append(("subscribe", self.name))
        self.hand.callbacks[self.name] = callback
        return self

    def publish(self):
        self.hand.calls.append(("publish", self.name))
        return self

    def send(self, frame): self.hand.calls.append(("send", frame))


class FakeHand:
    def __init__(self, side="left", firmware="2.5.1"):
        self.serial_number = "TEST-ONLY-NO-HARDWARE-DEVICE"
        self.info = SimpleNamespace(firmware_version=firmware)
        self.side = side
        self.calls, self.callbacks = [], {}

    def handedness(self): return Resource(self, "handedness", self.side)
    def joint_states(self): return Resource(self, "states")
    def joint_diagnostics(self): return Resource(self, "diagnostics")
    def joint_command(self): return Resource(self, "command")
    def effort_limit(self): return Resource(self, "effort_limit")
    def mit_params(self): return Resource(self, "mit_params")
    def disable(self): self.calls.append(("disable",))
    def enable(self): self.calls.append(("enable",))
    def emergency_stop(self): self.calls.append(("emergency_stop",))
    def clear_fault(self): self.calls.append(("clear_fault",))
    def disconnect(self): self.calls.append(("disconnect",))


def fake_sdk(hand):
    calls = []

    class Manager:
        @staticmethod
        def instance(): return Manager()

        def connect(self, **kwargs):
            calls.append(kwargs)
            return hand

    return SimpleNamespace(SdkManager=Manager, WujiHand2=FakeHand,
                           # Deliberately distinct enum tokens: handedness().get()
                           # returns a string in the real SDK, not these tokens.
                           Handedness=SimpleNamespace(Left=object(), Right=object()),
                           JointCommand=lambda *args: args), calls


def state_frame(seq=1, missing=False):
    entries = [SimpleNamespace(nid=bus * 5 + node, position=(bus * 4 + node) / 100,
                               velocity=0.0, effort=42.0)
               for bus in range(5) for node in range(1, 5)]
    if missing:
        entries.pop()
    return SimpleNamespace(header=SimpleNamespace(timestamp_us=seq * 1000, seq=seq),
                           joints=entries[::-1])


def test_explicit_identity_checked_before_any_write():
    for hand in [FakeHand(side="right"), FakeHand(firmware="2.4.0")]:
        sdk, calls = fake_sdk(hand)
        backend = SdkBackend(hand.serial_number, "left", sdk=sdk)
        with pytest.raises(RuntimeError):
            backend.connect()
        assert calls[0]["sn"] == hand.serial_number
        assert set(calls[0]) == {"sn", "device_name"}
        assert not any(c[0] in ("set", "enable", "disable", "publish", "send") for c in hand.calls)
        assert hand.calls[-1] == ("disconnect",)


def test_readonly_subscribes_and_disconnects_without_any_device_write():
    hand = FakeHand()
    sdk, calls = fake_sdk(hand)
    backend = SdkBackend(hand.serial_number, "left", read_only=True, sdk=sdk)
    backend.connect()
    try:
        for action in [backend.enable, backend.disable, backend.emergency_stop,
                       backend.clear_fault, lambda: backend.send([0.0] * 20),
                       lambda: backend.configure(3.0, 0.05, 1.5)]:
            with pytest.raises(RuntimeError, match="read_only"):
                action()
    finally:
        backend.close()
    assert not any(c[0] in ("set", "enable", "disable", "publish", "send") for c in hand.calls)


def test_callbacks_only_cache_scattered_full_frames_and_duplicates_do_not_refresh_age():
    hand = FakeHand()
    sdk, calls = fake_sdk(hand)
    clock = [1.0]
    backend = SdkBackend(hand.serial_number, "left", sdk=sdk, monotonic=lambda: clock[0])
    backend.connect()
    try:
        actions_before = len(hand.calls)
        hand.callbacks["states"](state_frame())
        assert backend.snapshot().state.positions == tuple(i / 100 for i in range(1, 21))
        assert backend.snapshot().state.received_at == 1.0
        clock[0] = 2.0
        hand.callbacks["states"](state_frame())
        assert backend.snapshot().state.received_at == 1.0
        hand.callbacks["states"](state_frame(seq=2, missing=True))
        assert backend.snapshot().error
        assert backend.snapshot().state.received_at == 1.0
        hand.callbacks["states"](state_frame(seq=3))
        assert not backend.snapshot().error
        assert backend.snapshot().state.received_at == 2.0
        assert len(hand.calls) == actions_before
    finally:
        backend.close()


def test_device_owner_rejects_duplicate_claim_and_releases():
    first = DeviceOwner("UNIT-TEST-OWNER")
    second = DeviceOwner("UNIT-TEST-OWNER")
    first.acquire()
    try:
        with pytest.raises(RuntimeError, match="already owns"):
            second.acquire()
    finally:
        first.release()
    second.acquire()
    second.release()


def test_non_owner_thread_cannot_write_sdk():
    hand = FakeHand()
    sdk, calls = fake_sdk(hand)
    backend = SdkBackend(hand.serial_number, "left", sdk=sdk)
    backend.connect()
    errors = []

    def other_thread():
        try:
            backend.enable()
        except RuntimeError as exc:
            errors.append(str(exc))

    try:
        worker = threading.Thread(target=other_thread)
        worker.start()
        worker.join()
        assert errors and "owner thread" in errors[0]
        assert not any(c[0] == "enable" for c in hand.calls)
    finally:
        backend.close()
