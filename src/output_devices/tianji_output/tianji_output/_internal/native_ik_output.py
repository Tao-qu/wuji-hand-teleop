"""Remove known successful kinematics debug output from the legacy library."""

import ctypes
import logging
import os
import re
import sys
import threading


_NUMBER = rb"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
_ROUTINE_LINE = re.compile(
    rb"(?:\[FxRobot - (?:FX_Robot_Kine_IK(?:_NSP)?|LOADMvCfg"
    rb"|FX_Robot_Init_Type|FX_Robot_Init_Kine|FX_Init_Robot_Kine_Pilot_CCS"
    rb"|FX_Robot_Kine_Piolt|FX_Robot_Init_Lmt|FX_Init_Robot_Lmt_CCS"
    rb"|FX_Robot_Tool_Set)\]"
    rb"|EG:ret_J=\[" + _NUMBER + rb"(?: " + _NUMBER + rb"){6}\]"
    rb"|j7=" + _NUMBER + rb" line:" + _NUMBER + rb")"
)
# All Marvin_Kine instances share the descriptor and its capture lock.
_LOCK = threading.RLock()
_CAPTURE = None
_FFLUSH = None
_C_STDOUT = None
_UNAVAILABLE = False


def _capture_file():
    global _CAPTURE, _FFLUSH, _C_STDOUT, _UNAVAILABLE
    if _UNAVAILABLE:
        return None
    if _CAPTURE is None:
        try:
            libc = ctypes.CDLL(None)
            _FFLUSH = libc.fflush
            _FFLUSH.argtypes = [ctypes.c_void_p]
            _FFLUSH.restype = ctypes.c_int
            _C_STDOUT = ctypes.c_void_p.in_dll(libc, "stdout")
            # Reused at control rate; no disk I/O, pipe capacity or reader thread.
            _CAPTURE = os.fdopen(os.memfd_create("tianji-ik-output", os.MFD_CLOEXEC), "w+b")
        except (AttributeError, OSError, ValueError) as exc:
            _UNAVAILABLE = True
            logging.getLogger(__name__).warning("Native IK output filter unavailable: %s", exc)
    return _CAPTURE


def _flush_stdout():
    try:
        sys.stdout.flush()
    finally:
        _FFLUSH(_C_STDOUT)


def _write_all(fd, data):
    view = memoryview(data)
    while view:
        view = view[os.write(fd, view):]


def call_with_filtered_stdout(function, *args):
    """Call native kinematics unchanged, forwarding all but known routine output.

    stdout is redirected only while this call runs; stderr is never redirected.
    Other threads' unmatched stdout is forwarded too. Failed calls retain their
    complete output, including the routine lines that may help diagnose them.
    """
    if sys.platform != "linux":
        return function(*args)
    with _LOCK:
        capture = _capture_file()
        if capture is None:
            return function(*args)
        _flush_stdout()
        capture.seek(0)
        capture.truncate()
        saved_stdout = os.dup(1)
        succeeded = False
        try:
            os.dup2(capture.fileno(), 1)
            result = function(*args)
            succeeded = bool(result)
            return result
        finally:
            try:
                try:
                    _flush_stdout()
                finally:
                    os.dup2(saved_stdout, 1)
                capture.seek(0)
                for line in capture:
                    if not succeeded or not _ROUTINE_LINE.fullmatch(line.rstrip(b"\r\n")):
                        _write_all(saved_stdout, line)
            finally:
                os.close(saved_stdout)
