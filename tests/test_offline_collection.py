"""Collection-policy regressions using isolated, harmless sentinel fixtures only.

Child pytest processes disable plugin autoload and never see the real repository
test trees, scratch scripts, ROS launch plugins, robot modules, or SDKs.
"""
import os
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
TEST_DIRS = (
    "tests", "src/wuji_hand2_driver/test", "src/wuji_teleop_monitor/tests",
    "src/input_devices/manus_input/manus_ros2/tools/tests",
)
BLOCKED = ("work", "build", "install", "log", "fetched", "retired", "deps",
           "src/wuji-description", "src/input_devices/manus_input/manus_ros2/ManusSDK")


def sentinel_module(path, marker):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f"from pathlib import Path\nPath({str(marker)!r}).write_text('imported')\n"
        "def test_sentinel():\n    pass\n", encoding="utf-8",
    )


@pytest.fixture
def sandbox(tmp_path):
    for name in ("pytest.ini", "conftest.py"):
        (tmp_path / name).write_text((ROOT / name).read_text(encoding="utf-8"), encoding="utf-8")
    for index, relative in enumerate(TEST_DIRS):
        directory = tmp_path / relative
        directory.mkdir(parents=True)
        (directory / f"test_synthetic_{index}.py").write_text(
            "def test_offline(pytestconfig):\n"
            "    assert pytestconfig.pluginmanager.is_blocked('launch_testing')\n"
            "    assert pytestconfig.pluginmanager.is_blocked('launch_ros')\n",
            encoding="utf-8",
        )
    marker = tmp_path / "unexpected-import.txt"
    for relative in BLOCKED:
        sentinel_module(tmp_path / relative / "test_sentinel.py", marker)
        sentinel_module(tmp_path / relative / "legacy_motion_test.py", marker)
    sentinel_module(tmp_path / "test_unapproved.py", marker)
    # The former broad *_test.py pattern must not collect this harmless sibling.
    sentinel_module(tmp_path / "tests/legacy_motion_test.py", marker)
    return tmp_path, marker


def run_pytest(directory, *args):
    env = os.environ.copy()
    for name in ("PYTEST_ADDOPTS", "PYTEST_PLUGINS", "PYTHONPATH"):
        env.pop(name, None)
    env["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    return subprocess.run(
        [sys.executable, "-m", "pytest", "-q", *args], cwd=directory,
        env=env, capture_output=True, text=True, timeout=30,
    )


@pytest.mark.parametrize("args", [(), (".",), ("--collect-only",), ("-k", "offline")])
def test_default_and_broad_collection_stay_in_approved_trees(sandbox, args):
    directory, marker = sandbox
    result = run_pytest(directory, *args)
    assert result.returncode == 0, result.stdout + result.stderr
    assert ("4 tests collected" if "--collect-only" in args else "4 passed") in result.stdout
    assert not marker.exists()


def test_explicit_approved_node_id_works(sandbox):
    directory, marker = sandbox
    result = run_pytest(directory, "tests/test_synthetic_0.py::test_offline", "-k", "offline")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "1 passed" in result.stdout
    assert not marker.exists()


def test_approved_tree_preserves_builtin_ignore_rules(sandbox):
    directory, marker = sandbox
    sentinel_module(directory / 'tests/test_ignored.py', marker)
    sentinel_module(directory / 'tests/.hidden/test_hidden.py', marker)
    result = run_pytest(directory, '--ignore=tests/test_ignored.py')
    assert result.returncode == 0, result.stdout + result.stderr
    assert '4 passed' in result.stdout
    assert not marker.exists()


@pytest.mark.parametrize("relative", BLOCKED)
def test_explicit_excluded_node_is_rejected_before_import(sandbox, relative):
    directory, marker = sandbox
    result = run_pytest(directory, "--collect-only", f"{relative}/test_sentinel.py::test_sentinel")
    assert result.returncode == 4, result.stdout + result.stderr
    assert "Offline collection rejected path" in result.stderr
    assert not marker.exists()


@pytest.mark.parametrize("absolute", [False, True])
def test_excluded_directory_is_rejected_before_import(sandbox, absolute):
    directory, marker = sandbox
    selected = str(directory / "work") if absolute else "work"
    result = run_pytest(directory, selected, "-k", "nothing")
    assert result.returncode == 4, result.stdout + result.stderr
    assert "Offline collection rejected path" in result.stderr
    assert not marker.exists()


def test_symlink_into_work_is_ignored_or_rejected(sandbox):
    directory, marker = sandbox
    link = directory / "tests/test_link.py"
    link.symlink_to(directory / "work/test_sentinel.py")
    normal = run_pytest(directory)
    assert normal.returncode == 0, normal.stdout + normal.stderr
    assert "4 passed" in normal.stdout
    explicit = run_pytest(directory, "tests/test_link.py::test_sentinel")
    assert explicit.returncode == 4, explicit.stdout + explicit.stderr
    assert "Offline collection rejected path" in explicit.stderr
    assert not marker.exists()


def test_rejected_path_never_reaches_module_collection_hook(sandbox):
    directory, marker = sandbox
    hook_marker = directory / "collection-hook-called.txt"
    # A harmless observer checks ordering without loading a ROS launch plugin.
    (directory / "observer.py").write_text(
        "from pathlib import Path\n"
        "def pytest_pycollect_makemodule(module_path, parent):\n"
        f"    Path({str(hook_marker)!r}).write_text('collection started')\n",
        encoding="utf-8",
    )
    result = run_pytest(directory, "-p", "observer", "work/test_sentinel.py")
    assert result.returncode == 4, result.stdout + result.stderr
    assert "Offline collection rejected path" in result.stderr
    assert not hook_marker.exists()
    assert not marker.exists()
