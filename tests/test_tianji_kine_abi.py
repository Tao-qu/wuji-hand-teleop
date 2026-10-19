"""Exercise bundled libKine memory layout and FK/IK without ROS or hardware I/O."""
import ctypes
from pathlib import Path
import platform
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/output_devices/tianji_output"))
from tianji_output._internal.fx_kine import Marvin_Kine
from tianji_output._internal.robot_structures import FX_InvKineSolvePara


def test_bundled_linux_ik_layout():
    if platform.system() != "Linux" or ctypes.sizeof(ctypes.c_long) != 8:
        pytest.skip("This workspace ships the Linux LP64 libKine ABI")
    assert ctypes.sizeof(FX_InvKineSolvePara) == 472
    for field, offset in {
        "m_Input_IK_ZSPType": 184,
        "m_Output_RetJoint": 272,
        "m_Output_IsOutRange": 328,
        "m_Output_JntExdABS": 344,
        "m_Output_IsJntExd": 352,
        "m_Output_RunLmtP": 360,
        "m_Output_RunLmtN": 416,
    }.items():
        assert getattr(FX_InvKineSolvePara, field).offset == offset


@pytest.mark.parametrize("serial", [0, 1])
def test_native_fk_ik_keeps_canaries_and_reports_limits(serial):
    lib = ROOT / "src/output_devices/tianji_output/tianji_output/_internal/lib/libKine.so"
    if platform.system() != "Linux" or platform.machine() not in ("x86_64", "AMD64"):
        pytest.skip("Bundled native library requires Linux x86_64")
    if not lib.is_file() or lib.read_bytes()[:4] != b"\x7fELF":
        pytest.skip("Fetch bundled libKine.so with Git LFS")

    # Oversized owner prevents a bad ABI from corrupting the test process.
    # Every byte outside the declared structure must remain untouched.
    guard = 64
    allocation = (ctypes.c_ubyte * 4096)()
    ctypes.memset(ctypes.addressof(allocation), 0xA5, ctypes.sizeof(allocation))
    kine = Marvin_Kine()  # libKine only: no control SDK, socket, or device session.
    kine.sp = FX_InvKineSolvePara.from_buffer(allocation, guard)
    ctypes.memset(ctypes.addressof(kine.sp), 0, ctypes.sizeof(kine.sp))
    cfg = kine.load_config(str(ROOT / "src/output_devices/tianji_output/tianji_output/config/ccs_m6.MvKDCfg"))
    assert kine.initial_kine(serial, cfg["TYPE"][serial], cfg["DH"][serial],
                             cfg["PNVA"][serial], cfg["BD"][serial])
    assert kine.set_tool_kine(serial, np.eye(4).tolist())
    for elbow in (-45.0, -60.0, -90.0):
        joints = [0.0, -20.0, 10.0, elbow, 0.0, 0.0, 0.0]
        target = kine.fk(serial, joints)
        result = kine.ik(serial, target, joints, 0, [0.0] * 6, 0.0, [5.0] * 3)
        assert result is not False
        output = result.m_Output_RetJoint.to_list()
        assert np.all(np.isfinite(output))
        assert np.allclose(kine.fk(serial, output), target, atol=1e-3)
        assert not result.m_Output_IsOutRange
        positive = np.asarray(result.m_Output_RunLmtP.to_list())
        negative = np.asarray(result.m_Output_RunLmtN.to_list())
        assert np.all(np.isfinite(positive)) and np.all(np.isfinite(negative))
        assert np.all(positive > negative)
        assert bool(result.m_Output_IsJntExd) == bool(np.any(np.asarray(output) > positive) or np.any(np.asarray(output) < negative))
        assert bytes(allocation[:guard]) == b"\xa5" * guard
        tail = guard + ctypes.sizeof(kine.sp)
        assert bytes(allocation[tail:]) == b"\xa5" * (len(allocation) - tail)
