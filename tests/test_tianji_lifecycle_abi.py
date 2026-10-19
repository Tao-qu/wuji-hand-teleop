"""Native callback fixtures only: never load a robot library or connect."""
import ctypes
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/output_devices/tianji_output'))
from tianji_output._internal.fx_robot import Marvin_Robot


@pytest.mark.parametrize('register_value,expected', [(0x12340001, True), (0x12340000, False)])
def test_release_uses_only_native_bool_return_byte(register_value, expected):
    # C++ bool callers must not interpret unspecified upper return-register bits.
    function = ctypes.CFUNCTYPE(ctypes.c_int)(lambda: register_value)
    owner = Marvin_Robot.__new__(Marvin_Robot)
    owner.robot = SimpleNamespace(OnRelease=function)
    assert owner.release_robot() is expected


def test_sdk_version_preserves_native_long_width():
    if ctypes.sizeof(ctypes.c_long) != 8:
        pytest.skip('LP64 native library in this Linux installation')
    value = 0x100000001
    function = ctypes.CFUNCTYPE(ctypes.c_long)(lambda: value)
    function.restype = ctypes.c_int  # ctypes' unbound default would truncate.
    owner = Marvin_Robot.__new__(Marvin_Robot)
    owner.robot = SimpleNamespace(OnGetSDKVersion=function)
    assert owner.SDK_version() == value
