"""Restrict ordinary pytest collection to the project's offline unit-test trees.

This prevents collecting operational scratch scripts; it is not a sandbox for
code deliberately imported by an approved test or an explicitly loaded plugin.
"""
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parent
TEST_DIRS = tuple(ROOT / path for path in (
    "tests",
    "src/wuji_hand2_driver/test",
    "src/wuji_teleop_monitor/tests",
    "src/input_devices/manus_input/manus_ros2/tools/tests",
))
EXCLUDED = frozenset((
    "work", "build", "install", "log", "fetched", "retired", "deps",
    "vendor", "external", "third_party", "wuji-description", "ManusSDK",
    "retargeting", "wujihandros2",
))


def _allowed(path):
    path = Path(path).resolve()
    if not path.is_relative_to(ROOT):
        return False
    if EXCLUDED.intersection(path.relative_to(ROOT).parts):
        return False
    # Ancestors permit `pytest .` / `pytest src`; the collection hook then
    # follows only branches leading to an approved test directory.
    return any(path.is_relative_to(test_dir) or test_dir.is_relative_to(path)
               for test_dir in TEST_DIRS)


@pytest.hookimpl(tryfirst=True)
def pytest_configure(config):
    if config.getoption("pyargs", default=False):
        raise pytest.UsageError("Offline collection requires filesystem test paths, not --pyargs")
    for name in ("launch_testing", "launch_ros"):
        if config.pluginmanager.hasplugin(name):
            raise pytest.UsageError(f"Offline collection requires disabling the {name} plugin")
    for selector in config.args:
        # Strip the node ID without importing a module to resolve it.
        path = Path(str(selector).split("::", 1)[0])
        if not path.is_absolute():
            path = Path(config.invocation_params.dir) / path
        if not _allowed(path):
            raise pytest.UsageError(
                f"Offline collection rejected path: {selector}. "
                "Use one of the four testpaths in pytest.ini; operational and dependency trees are excluded."
            )


@pytest.hookimpl(tryfirst=True)
def pytest_ignore_collect(collection_path, config):
    # Also resolves symlinks so an approved directory cannot expose work/.
    # Let pytest's built-in --ignore/hidden-directory handling decide for an
    # approved path; returning False would suppress later first-result hooks.
    return True if not _allowed(collection_path) else None
