"""Read-only input statistics and non-overwriting configuration tests."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.configure import CONFIG_TEMPLATES, configure
from tools.sample_ros_inputs import StreamStats


class InputStatisticsTests(unittest.TestCase):
    def test_frozen_sequence_does_not_renew_freshness(self):
        stats = StreamStats(.2)
        stats.observe(receipt=0., ros_now_ns=1_000_000_000, stamp_ns=1_000_000_000, sequence=1)
        # Even a wrongly updated ROS stamp cannot hide an unchanged sequence.
        stats.observe(receipt=.3, ros_now_ns=1_300_000_000, stamp_ns=1_300_000_000, sequence=1)
        report = stats.report(receipt_now=.31, ros_now_ns=1_310_000_000, duration=.31)
        self.assertEqual(report['frames'], 2)
        self.assertEqual(report['repeated_sequences'], 1)
        self.assertFalse(report['fresh_at_end'])

    def test_reports_rates_gaps_and_old_source_frames(self):
        stats = StreamStats(.2)
        stats.observe(receipt=0., ros_now_ns=1_000_000_000, stamp_ns=990_000_000, sequence=2)
        stats.observe(receipt=.01, ros_now_ns=1_010_000_000, stamp_ns=1_000_000_000, sequence=5)
        report = stats.report(receipt_now=.02, ros_now_ns=1_020_000_000, duration=.02)
        self.assertAlmostEqual(report['stream_hz'], 100.)
        self.assertEqual(report['sequence_gaps'], 2)
        self.assertTrue(report['fresh_at_end'])
        stats.observe(receipt=.5, ros_now_ns=1_500_000_000, stamp_ns=1_001_000_000, sequence=6)
        report = stats.report(receipt_now=.5, ros_now_ns=1_500_000_000, duration=.5)
        self.assertEqual(report['source_stale_frames'], 1)
        self.assertFalse(report['fresh_at_end'])

    def test_static_transforms_do_not_expire(self):
        stats = StreamStats(.2, static=True)
        stats.observe(receipt=0., ros_now_ns=5_000_000_000, stamp_ns=1_000_000_000)
        report = stats.report(receipt_now=10., ros_now_ns=15_000_000_000, duration=10.)
        self.assertTrue(report['fresh_at_end'])
        self.assertEqual(report['source_stale_frames'], 0)

    def test_rejected_future_data_does_not_block_following_valid_frames(self):
        stats = StreamStats(.2)
        stats.observe(receipt=0., ros_now_ns=1_000_000_000, stamp_ns=9_000_000_000, sequence=99)
        stats.observe(receipt=.01, ros_now_ns=1_010_000_000, stamp_ns=1_000_000_000, sequence=2)
        report = stats.report(receipt_now=.02, ros_now_ns=1_020_000_000, duration=.02)
        self.assertTrue(report['fresh_at_end'])
        self.assertEqual(report['source_future_frames'], 1)

    def test_missing_stream_and_invalid_stamp_are_not_fresh(self):
        stats = StreamStats()
        self.assertFalse(stats.report(receipt_now=1., ros_now_ns=1_000_000_000, duration=1.)['fresh_at_end'])
        stats.observe(receipt=1., ros_now_ns=1_000_000_000, stamp_ns=0, sequence=0)
        report = stats.report(receipt_now=1., ros_now_ns=1_000_000_000, duration=1.)
        self.assertEqual(report['invalid_stamps'], 1)
        self.assertEqual(report['invalid_sequences'], 1)
        self.assertFalse(report['fresh_at_end'])


class ConfigurationTests(unittest.TestCase):
    def test_only_own_templates_and_existing_settings_are_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for relative in CONFIG_TEMPLATES:
                template = root / relative
                template.parent.mkdir(parents=True, exist_ok=True)
                template.write_text('setting: template\n')
            external = root / 'src/external/config/unused.yaml.template'
            external.parent.mkdir(parents=True)
            external.write_text('external: true\n')
            self.assertEqual(len(configure(root, check_only=True)), 3)
            self.assertFalse((root / CONFIG_TEMPLATES[0]).with_suffix('').exists())
            self.assertEqual(len(configure(root)), 3)
            local = (root / CONFIG_TEMPLATES[0]).with_suffix('')
            local.write_text('setting: operator\n')
            self.assertEqual(configure(root), [])
            self.assertEqual(local.read_text(), 'setting: operator\n')
            self.assertFalse(external.with_suffix('').exists())


if __name__ == '__main__':
    unittest.main()
