"""Exercise the runner in temporary harmless repositories, never work scripts."""
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


@pytest.fixture
def runner_repo(tmp_path):
    repo = tmp_path / "repo"
    (repo / "tools").mkdir(parents=True)
    (repo / "tools/test_offline.py").write_text(
        (ROOT / "tools/test_offline.py").read_text(encoding="utf-8"), encoding="utf-8")
    (repo / "pytest.ini").write_text("[pytest]\npython_files = test_*.py\n", encoding="utf-8")
    root_marker = repo / "root-conftest-imported.txt"
    blocked_marker = repo / "blocked-conftest-imported.txt"
    plugin_marker = repo / "plugin-imported.txt"
    (repo / "conftest.py").write_text(
        f"from pathlib import Path\nPath({str(root_marker)!r}).write_text('imported')\n",
        encoding="utf-8")
    for index, relative in enumerate(TEST_DIRS):
        directory = repo / relative
        directory.mkdir(parents=True)
        (directory / f"test_synthetic_{index}.py").write_text(
            "import os\nimport trusted_runtime_sentinel\n"
            "def test_offline(pytestconfig):\n"
            "    assert os.environ['PYTEST_DISABLE_PLUGIN_AUTOLOAD'] == '1'\n"
            "    assert 'PYTEST_PLUGINS' not in os.environ\n"
            "    assert 'PYTEST_ADDOPTS' not in os.environ\n"
            "    assert pytestconfig.inipath.name == 'pytest.ini'\n"
            "    assert str(pytestconfig.rootpath) == os.getcwd()\n",
            encoding="utf-8")
    work = repo / "work"
    work.mkdir()
    (work / "conftest.py").write_text(
        f"from pathlib import Path\nPath({str(blocked_marker)!r}).write_text('imported')\n"
        "raise RuntimeError('Unapproved conftest must never be imported')\n",
        encoding="utf-8")
    (work / "test_danger.py").write_text("raise RuntimeError('must not be imported')\n", encoding="utf-8")
    runtime = tmp_path / "trusted-runtime"
    runtime.mkdir()
    (runtime / "trusted_runtime_sentinel.py").write_text("VALUE = 'preserved'\n", encoding="utf-8")
    (runtime / "unexpected_plugin.py").write_text(
        f"from pathlib import Path\nPath({str(plugin_marker)!r}).write_text('imported')\n",
        encoding="utf-8")
    return repo, runtime, root_marker, blocked_marker, plugin_marker


def invoke(fixture, *args, cwd=None):
    repo, runtime, *_ = fixture
    env = os.environ.copy()
    # No real repository Python path is exposed to the child pytest process.
    env['PYTHONPATH'] = str(runtime)
    env['PYTEST_PLUGINS'] = 'unexpected_plugin'
    env['PYTEST_ADDOPTS'] = '--this-environment-option-must-be-removed'
    return subprocess.run(
        [sys.executable, str(repo / 'tools/test_offline.py'), *args],
        cwd=cwd or repo.parent, env=env, capture_output=True, text=True, timeout=30)


@pytest.mark.parametrize('selector', ['work', 'work/test_danger.py::test_anything',
                                     'tests/../work/test_danger.py'])
def test_unapproved_selector_rejected_before_any_conftest_import(runner_repo, selector):
    repo, _, root_marker, blocked_marker, plugin_marker = runner_repo
    result = invoke(runner_repo, selector, cwd=repo / 'work')
    assert result.returncode == 2, result.stdout + result.stderr
    assert 'Offline preflight rejected' in result.stderr
    assert not root_marker.exists() and not blocked_marker.exists() and not plugin_marker.exists()


def test_external_selector_rejected_before_pytest_start(runner_repo, tmp_path):
    outside = tmp_path / 'test_external.py'
    outside.write_text("raise RuntimeError('must not be imported')\n", encoding='utf-8')
    result = invoke(runner_repo, str(outside))
    assert result.returncode == 2 and 'External test path' in result.stderr
    assert not runner_repo[2].exists()


@pytest.mark.parametrize('directory', [False, True])
def test_symlink_cannot_expose_work_conftest_during_startup(runner_repo, directory):
    repo, _, root_marker, blocked_marker, _ = runner_repo
    link = repo / ('tests/test_link' if directory else 'tests/test_link.py')
    link.symlink_to(repo / ('work' if directory else 'work/test_danger.py'), target_is_directory=directory)
    result = invoke(runner_repo, 'tests')
    assert result.returncode == 2 and 'Symlink' in result.stderr
    assert not root_marker.exists() and not blocked_marker.exists()


def test_nonroot_cwd_and_nested_package_config_cannot_override_root_policy(runner_repo):
    repo, _, root_marker, blocked_marker, plugin_marker = runner_repo
    package = repo / 'src/wuji_hand2_driver'
    (package / 'setup.cfg').write_text(
        '[tool:pytest]\naddopts = --this-package-option-must-not-be-loaded\n', encoding='utf-8')
    result = invoke(runner_repo, '-q', 'src/wuji_hand2_driver/test/test_synthetic_1.py::test_offline', cwd=package)
    assert result.returncode == 0 and '1 passed' in result.stdout, result.stdout + result.stderr
    assert root_marker.exists() and not blocked_marker.exists() and not plugin_marker.exists()


def test_default_and_collect_only_use_only_approved_paths(runner_repo):
    result = invoke(runner_repo, '-q', '--collect-only', '-k', 'offline')
    assert result.returncode == 0 and '4 tests collected' in result.stdout, result.stdout + result.stderr
    assert not runner_repo[3].exists() and not runner_repo[4].exists()


@pytest.mark.parametrize('arguments', [('-p', 'unexpected_plugin'), ('-c', 'other.ini'),
                                      ('--confcutdir', 'work'), ('--pyargs', 'unexpected_plugin')])
def test_arbitrary_pytest_plugin_or_config_flags_are_not_forwarded(runner_repo, arguments):
    result = invoke(runner_repo, *arguments)
    assert result.returncode == 2, result.stdout + result.stderr
    assert not runner_repo[2].exists() and not runner_repo[4].exists()
