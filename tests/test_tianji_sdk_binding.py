"""Exercise native argument sizes and read-only connection without a device."""
import ctypes
from types import SimpleNamespace

import pytest

from tianji_output._internal.fx_robot import Marvin_Robot


class NativeFunction:
    def __init__(self, function):
        self.function = function

    def __call__(self, *args):
        return self.function(*args)


class NativeSdk:
    def __init__(self, linked=True):
        self.calls = []
        self.OnLinkTo = NativeFunction(lambda *args: self.link(linked, args))
        self.OnGetIntPara = NativeFunction(self.get_int)

    def link(self, result, args):
        self.calls.append(("link", tuple(x.value for x in args)))
        return result

    def get_int(self, name, result):
        assert ctypes.sizeof(name) == 30
        assert name.raw[-1] == 0
        # The vendor implementation writes a full native long on Linux.
        assert ctypes.sizeof(result._obj) == ctypes.sizeof(ctypes.c_long)
        ctypes.cast(result, ctypes.POINTER(ctypes.c_long))[0] = 100343009
        self.calls.append(("get", name.value.decode()))
        return 0


def test_connect_uses_only_read_only_handshake_and_full_native_buffers(monkeypatch):
    native = NativeSdk()
    monkeypatch.delenv("TIANJI_SDK_LIBRARY", raising=False)
    monkeypatch.setattr(ctypes, "CDLL", lambda path: native)
    robot = Marvin_Robot()
    assert robot.connect("6.6.7.190")
    assert robot.controller_version == 100343009
    assert native.calls == [("link", (6, 6, 7, 190)), ("get", "VERSION")]
    assert robot.get_param("int", "VERSION") == (0, 100343009)


def test_failed_link_does_not_issue_queries(monkeypatch):
    native = NativeSdk(linked=False)
    monkeypatch.delenv("TIANJI_SDK_LIBRARY", raising=False)
    monkeypatch.setattr(ctypes, "CDLL", lambda path: native)
    assert not Marvin_Robot().connect("6.6.7.190")
    assert native.calls == [("link", (6, 6, 7, 190))]


@pytest.mark.parametrize('arm', ['A', 'B'])
@pytest.mark.parametrize('returned', [False, True])
def test_mode_command_wrappers_declare_vendor_bool_and_argument_widths(arm, returned):
    pointer = ctypes.POINTER(ctypes.c_double)
    cases = [
        ('clear_set', {}, 'OnClearSet', []),
        ('send_cmd', {}, 'OnSetSend', []),
        ('set_state', {'arm': arm, 'state': 3}, f'OnSetTargetState_{arm}', [ctypes.c_int]),
        ('set_impedance_type', {'arm': arm, 'type': 1}, f'OnSetImpType_{arm}', [ctypes.c_int]),
        ('set_vel_acc', {'arm': arm, 'velRatio': 10, 'AccRatio': 10},
         f'OnSetJointLmt_{arm}', [ctypes.c_int, ctypes.c_int]),
        ('set_joint_cmd_pose', {'arm': arm, 'joints': [1.] * 7},
         f'OnSetJointCmdPos_{arm}', [pointer]),
        ('set_joint_kd_params', {'arm': arm, 'K': [2.] * 7, 'D': [.3] * 7},
         f'OnSetJointKD_{arm}', [pointer, pointer]),
        ('set_cart_kd_params', {'arm': arm, 'K': [2.] * 7, 'D': [.3] * 7, 'type': 2},
         f'OnSetCartKD_{arm}', [pointer, pointer, ctypes.c_int]),
    ]
    robot = Marvin_Robot.__new__(Marvin_Robot)  # No CDLL load or device connection.
    for method, arguments, native_name, types in cases:
        seen = []
        function = NativeFunction(lambda *args: seen.append(args) or returned)
        robot.robot = SimpleNamespace(**{native_name: function})
        assert getattr(robot, method)(**arguments) is returned
        assert function.restype is ctypes.c_bool
        assert function.argtypes == types
        assert len(seen) == 1 and len(seen[0]) == len(types)
