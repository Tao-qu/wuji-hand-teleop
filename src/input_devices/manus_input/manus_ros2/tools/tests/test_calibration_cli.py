"""Compile the actual CLI against a fake SDK; no MANUS library or device is used."""
import os
from pathlib import Path
import shutil
import selectors
import signal
import time
import subprocess
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parents[1]
SDK_HEADERS = TOOLS.parent / 'ManusSDK/include'
GLOVE_ID = '229813950'


@unittest.skipUnless(shutil.which('g++'), 'C++ compiler unavailable')
class CalibrationCliTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory(prefix='manus-cli-mock-')
        cls.binary = Path(cls.build.name) / 'mock-calibrate'
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-pthread',
                        '-I', str(SDK_HEADERS), str(TOOLS / 'manus_calibrate.cpp'),
                        str(TOOLS / 'tests/fake_manus_sdk.cpp'), '-o', str(cls.binary)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.build.cleanup()

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='manus-cli-case-')
        self.addCleanup(self.directory.cleanup)
        self.output = Path(self.directory.name) / 'new.mcal'
        self.trace = Path(self.directory.name) / 'calls.txt'

    def run_cli(self, *, calibrate=False, stdin='', mode='', extra=()):
        args = [str(self.binary), '--glove-id', GLOVE_ID, '--side', 'right']
        if calibrate:
            args += ['--calibrate', '--robots-disabled', '--output', str(self.output)]
        args += list(extra)
        env = dict(os.environ, MANUS_TEST_MODE=mode, MANUS_TEST_TRACE=str(self.trace))
        process = subprocess.run(args, input=stdin, text=True, capture_output=True, env=env, timeout=10)
        calls = self.trace.read_text().splitlines() if self.trace.exists() else []
        return process, calls

    def test_inspect_never_mutates_and_indexes_shuffled_ids(self):
        process, calls = self.run_cli()
        self.assertEqual(process.returncode, 0, process.stdout)
        self.assertIn('id=18 parent=17 side=2 chain=Pinky(9) joint=Proximal(2)', process.stdout)
        self.assertIn('id=25 parent=24', process.stdout)
        self.assertLess(process.stdout.index('id=1 parent'), process.stdout.index('id=18 parent'))
        self.assertIn('Step 1:', process.stdout)
        self.assertFalse(any(x.split()[0] in {'start', 'run_step', 'finish', 'stop', 'export_size'} for x in calls))
        self.assertEqual(calls[-1].split()[0], 'shutdown')

    def test_wrong_side_fails_before_calibration(self):
        process, calls = self.run_cli(calibrate=True, mode='wrong_side')
        self.assertEqual(process.returncode, 1)
        self.assertNotIn('start ' + GLOVE_ID, calls)
        self.assertFalse(self.output.exists())

    def test_existing_profile_rejected_before_sdk(self):
        self.output.write_bytes(b'old')
        process, calls = self.run_cli(calibrate=True)
        self.assertEqual(process.returncode, 1)
        self.assertFalse(calls)
        self.assertEqual(self.output.read_bytes(), b'old')

    def test_cancel_before_start_does_not_create_profile(self):
        process, calls = self.run_cli(calibrate=True, stdin='cancel\n')
        self.assertEqual(process.returncode, 2)
        self.assertNotIn('start ' + GLOVE_ID, calls)
        self.assertFalse(self.output.exists())

    def test_cancel_after_start_stops_without_finish(self):
        process, calls = self.run_cli(calibrate=True, stdin='start ' + GLOVE_ID + '\ncancel\n')
        self.assertEqual(process.returncode, 2)
        self.assertIn('stop ' + GLOVE_ID, calls)
        self.assertNotIn('finish ' + GLOVE_ID, calls)
        self.assertFalse(self.output.exists())

    def test_signals_cancel_waiting_prompt_without_newline(self):
        # Exercise the real input loop against the fake SDK, including a partial
        # command that would make std::getline block even after poll reports data.
        for sig, partial in [(signal.SIGINT, b''), (signal.SIGTERM, b'ste')]:
            with self.subTest(signal=sig, partial=partial):
                if self.trace.exists():
                    self.trace.unlink()
                args = [str(self.binary), '--glove-id', GLOVE_ID, '--side', 'right',
                        '--calibrate', '--robots-disabled', '--output', str(self.output)]
                env = dict(os.environ, MANUS_TEST_TRACE=str(self.trace), MANUS_TEST_MODE='')
                process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE, env=env)
                selector = selectors.DefaultSelector()
                selector.register(process.stdout, selectors.EVENT_READ)

                def wait_for_prompt(token):
                    received = b''
                    deadline = time.monotonic() + 5
                    while token not in received and time.monotonic() < deadline:
                        if selector.select(0.1):
                            chunk = os.read(process.stdout.fileno(), 4096)
                            if not chunk:
                                break
                            received += chunk
                    self.assertIn(token, received)

                try:
                    wait_for_prompt(b"Type 'start")
                    process.stdin.write(('start ' + GLOVE_ID + '\n').encode())
                    process.stdin.flush()
                    wait_for_prompt(b"Type 'step 0'")
                    if partial:
                        process.stdin.write(partial)
                        process.stdin.flush()
                    process.send_signal(sig)
                    self.assertEqual(process.wait(timeout=2), 2)
                    calls = self.trace.read_text().splitlines()
                    self.assertIn('stop ' + GLOVE_ID, calls)
                    self.assertNotIn('finish ' + GLOVE_ID, calls)
                    self.assertEqual(calls[-1], 'shutdown ' + GLOVE_ID)
                    self.assertFalse(self.output.exists())
                finally:
                    selector.close()
                    if process.poll() is None:
                        process.kill()
                        process.wait(timeout=5)
                    process.stdin.close()
                    process.stdout.close()
                    process.stderr.close()

    def test_each_return_layer_failure_stops_without_export(self):
        for mode in ['step_rejected', 'step_sdk_failure']:
            with self.subTest(mode=mode):
                if self.trace.exists(): self.trace.unlink()
                process, calls = self.run_cli(calibrate=True, mode=mode, stdin='start ' + GLOVE_ID + '\nstep 0\n')
                self.assertEqual(process.returncode, 1)
                self.assertIn('stop ' + GLOVE_ID, calls)
                self.assertNotIn('finish ' + GLOVE_ID, calls)
                self.assertFalse(self.output.exists())

    def test_success_only_selected_glove_and_exclusive_output(self):
        process, calls = self.run_cli(calibrate=True, stdin='start ' + GLOVE_ID + '\nstep 0\nstep 1\nfinish ' + GLOVE_ID + '\n')
        self.assertEqual(process.returncode, 0, process.stdout)
        self.assertEqual(self.output.read_bytes(), b'****')
        self.assertEqual(self.output.stat().st_mode & 0o777, 0o600)
        self.assertTrue(all(x.split()[1] == GLOVE_ID for x in calls))
        self.assertNotIn('stop ' + GLOVE_ID, calls)
        self.assertEqual([x.split()[0] for x in calls[-4:]], ['finish', 'export_size', 'export_bytes', 'shutdown'])

    def test_export_failure_reports_already_applied_and_removes_partial(self):
        process, calls = self.run_cli(calibrate=True, mode='export_failure', stdin='start ' + GLOVE_ID + '\nstep 0\nstep 1\nfinish ' + GLOVE_ID + '\n')
        self.assertEqual(process.returncode, 1)
        self.assertIn('Finish already applied', process.stdout)
        self.assertIn('finish ' + GLOVE_ID, calls)
        self.assertNotIn('stop ' + GLOVE_ID, calls)
        self.assertFalse(self.output.exists())

    def test_duplicate_node_id_rejects_hierarchy_before_start(self):
        process, calls = self.run_cli(calibrate=True, mode='duplicate')
        self.assertEqual(process.returncode, 1)
        self.assertIn('Duplicate MANUS hierarchy node ID', process.stdout)
        self.assertNotIn('start ' + GLOVE_ID, calls)


if __name__ == '__main__':
    unittest.main()
