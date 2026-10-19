#!/usr/bin/env python3
"""Run only approved offline tests after a stdlib-only path preflight.

Relative selectors are always relative to this repository, even when invoked
from another directory. The interpreter and PYTHONPATH must be trusted (for
example the ROS/runtime environment supplied by tools/env.sh); this is a
collection guard, not a sandbox for arbitrary test code or Python startup.
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
TEST_DIRS = (
    "tests",
    "src/wuji_hand2_driver/test",
    "src/wuji_teleop_monitor/tests",
    "src/input_devices/manus_input/manus_ros2/tools/tests",
)
EXCLUDED = frozenset((
    "work", "build", "install", "log", "fetched", "retired", "deps",
    "vendor", "external", "third_party", "wuji-description", "ManusSDK",
    "retargeting", "wujihandros2",
))


def _reject_symlinks(path):
    # Reject selected aliases before resolving them, including parent links.
    for candidate in (path, *path.parents):
        if candidate == ROOT:
            break
        if candidate.is_symlink():
            raise ValueError(f"Symlink test paths are not permitted: {candidate}")
    if path.is_dir():
        # Initial conftest loading precedes pytest's ignore hooks. In
        # particular, a test* directory symlink must not expose work/conftest.
        for directory, subdirs, files in os.walk(path, followlinks=False):
            for name in (*subdirs, *files):
                candidate = Path(directory) / name
                if candidate.is_symlink():
                    raise ValueError(f"Symlink inside selected test tree: {candidate}")


def checked_selector(selector):
    path_text, separator, node_id = selector.partition("::")
    if not path_text:
        raise ValueError("A filesystem test path is required")
    candidate = Path(path_text)
    if not candidate.is_absolute():
        candidate = ROOT / candidate
    resolved = candidate.resolve(strict=True)
    if not resolved.is_relative_to(ROOT):
        raise ValueError(f"External test path is not permitted: {selector}")
    relative = resolved.relative_to(ROOT)
    if EXCLUDED.intersection(relative.parts) or not any(
            resolved.is_relative_to(ROOT / approved) for approved in TEST_DIRS):
        raise ValueError(f"Unapproved offline test path: {selector}")
    if not (resolved.is_file() or resolved.is_dir()):
        raise ValueError(f"Not a regular test file or directory: {selector}")
    _reject_symlinks(candidate)
    return str(resolved) + (separator + node_id if separator else "")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("selectors", nargs="*", help="Approved test paths or path::node IDs")
    parser.add_argument("-q", action="count", default=0, help="Reduce pytest verbosity")
    parser.add_argument("--collect-only", action="store_true")
    parser.add_argument("-k", metavar="EXPRESSION", help="Filter approved tests by name")
    args = parser.parse_args(argv)
    try:
        selectors = [checked_selector(value) for value in (args.selectors or TEST_DIRS)]
        config = ROOT / "pytest.ini"
        if not config.is_file() or config.is_symlink():
            raise ValueError("Repository pytest.ini must be an existing regular file")
    except (OSError, ValueError, RuntimeError) as error:
        parser.error(f"Offline preflight rejected the request: {error}")

    # No pytest import or process startup is allowed above this point.
    command = [sys.executable, "-m", "pytest", "-c", str(config), "--confcutdir", str(ROOT)]
    command.extend(["-q"] * args.q)
    if args.collect_only:
        command.append("--collect-only")
    if args.k is not None:
        command.extend(["-k", args.k])
    command.append("--")
    command.extend(selectors)
    env = os.environ.copy()
    env["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    env.pop("PYTEST_PLUGINS", None)
    env.pop("PYTEST_ADDOPTS", None)
    # Preserve trusted ROS/runtime paths. Sanitizing PYTHONPATH here would
    # break the normal environment without preventing interpreter startup code.
    return subprocess.run(command, cwd=ROOT, env=env, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
