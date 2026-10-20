"""Interactive five-Tracker calibration for the configured Tianji arm Home."""

import argparse
from pathlib import Path
import signal
import time

from .tracker_calibration_io import (
    apply_profile,
    default_config,
    load_context,
    load_profile,
    make_profile,
    read_config,
    save_profile,
)


def _positive_seconds(value):
    import math

    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0:
        raise argparse.ArgumentTypeError("duration must be a positive number of seconds")
    return seconds


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def _parser():
    parser = argparse.ArgumentParser(
        description="Inspect or calibrate five Trackers against the configured arm Home.",
        epilog="Calibration reads OpenVR and local kinematics only. "
               "Apply writes configuration; restart teleoperation to load it.",
    )
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--inspect", action="store_true",
                       help="show configured Trackers and Home (default)")
    modes.add_argument("--calibrate", action="store_true",
                       help="capture a neutral pose and export a calibration")
    modes.add_argument("--apply", metavar="PROFILE", type=Path,
                       help="apply a saved calibration to the existing YAML files")
    parser.add_argument("--tracker-config", type=Path,
                        help="OpenVR YAML (default: installed openvr_input/config/openvr_input.yaml)")
    parser.add_argument("--tf-config", type=Path,
                        help="existing static_transforms.yaml; required for calibration and apply")
    parser.add_argument("--robot-config", type=Path,
                        help="Tianji YAML (default: installed tianji_output/config/tianji_chest.yaml)")
    parser.add_argument("--kinematics-config", type=Path,
                        help="kinematics model (default: installed tianji_output/config/ccs_m6.MvKDCfg)")
    parser.add_argument("--duration", type=_positive_seconds, default=5.0,
                        help="capture duration in seconds (default: 5)")
    parser.add_argument("--output", type=Path,
                        help="new calibration YAML to export; its parent directory must exist")
    return parser


def _show_devices(devices):
    print("\nConfigured Trackers:")
    for role, device in devices.items():
        print(f"  {role:12s} {device['serial']:16s} {device['status']}")


def _show_home(robot_config):
    print("\nReference Home joint angles (degrees):")
    for side in ("left", "right"):
        print(f"  {side}: {robot_config['init_joints'][side]}")


def _show_fit(result):
    fit = result["fit"]
    print("\nMirror-constrained fit relative to Home:")
    for side in ("left", "right"):
        values = fit[side]
        print(
            f"  {side}: position {values['mean_position_error_mm']:.3f} mm, "
            f"wrist {values['mean_wrist_angle_deg']:.3f} deg, "
            f"upper arm {values['mean_upper_direction_angle_deg']:.3f} deg"
        )


def _capture_and_fit(context, args):
    from .tracker_capture import TrackerCapture
    from .tracker_calibration import fit_calibration

    _show_home(context["robot_config"])
    with TrackerCapture(context["tracker_config"]["tracker_serials"]) as session:
        _show_devices(session.inspect())
        while True:
            print(
                "\nStand upright with both upper arms naturally spread, elbows bent, "
                "hands in front and palms down. Keep the pose symmetric and all five "
                "Trackers visible."
            )
            input("Press Enter when ready; capture starts after a 3-second countdown. ")
            for count in (3, 2, 1):
                print(f"Starting in {count}...", flush=True)
                time.sleep(1)
            print(f"Capture started. Hold the pose for {args.duration:g} seconds.", flush=True)
            samples, stats = session.capture(duration=args.duration)
            print("Capture complete. You can relax.", flush=True)
            print(f"Synchronized valid frames: {stats['valid_frames']}/{stats['iterations']}")
            print("Invalid frames by role: " + ", ".join(
                f"{role}={count}" for role, count in stats["invalid_counts"].items()))
            if not stats["valid_frames"]:
                choice = input("No complete samples. Enter 'retry' to capture again, or 'cancel': ")
                if choice.strip().lower() == "retry":
                    continue
                return 2
            result = fit_calibration(context, samples)
            _show_fit(result)
            while True:
                choice = input("Enter 'save', 'retry', or 'cancel': ").strip().lower()
                if choice in ("save", "retry", "cancel"):
                    break
            if choice == "retry":
                continue
            if choice == "cancel":
                return 2
            profile = make_profile(result, context["tracker_config"])
            path = save_profile(args.output, profile)
            print(f"Calibration exported to {path}")
            print("Use --apply with this profile and --tf-config to update the running configuration.")
            return 0


def main(argv=None):
    parser = _parser()
    args = parser.parse_args(argv)
    if (args.calibrate or args.apply) and args.tf_config is None:
        parser.error("--tf-config is required with --calibrate or --apply")
    if args.calibrate and args.output is None:
        parser.error("--output is required with --calibrate")
    if args.output is not None and not args.calibrate:
        parser.error("--output requires --calibrate")
    for name in ("tracker_config", "tf_config", "robot_config", "kinematics_config", "output", "apply"):
        value = getattr(args, name)
        if value is not None:
            setattr(args, name, value.expanduser())
    previous_sigterm = signal.signal(signal.SIGTERM, _interrupt)
    try:
        tracker_path = args.tracker_config or default_config("openvr_input", "openvr_input.yaml")
        if args.apply:
            profile = load_profile(args.apply)
            paths = apply_profile(profile, tracker_path, args.tf_config)
            for path in paths:
                print(f"Updated calibration fields: {path}")
            print("Restart teleoperation to load the applied calibration.")
            return 0

        robot_path = args.robot_config or default_config("tianji_output", "tianji_chest.yaml")
        if args.calibrate:
            if args.output.exists() or args.output.is_symlink():
                raise ValueError(f"Choose a new output file: {args.output}")
            if not args.output.parent.is_dir():
                raise ValueError(f"Output directory does not exist: {args.output.parent}")
            kinematics_path = args.kinematics_config or default_config("tianji_output", "ccs_m6.MvKDCfg")
            context = load_context(tracker_path, args.tf_config, robot_path, kinematics_path)
            return _capture_and_fit(context, args)

        from .tracker_capture import TrackerCapture

        tracker_config = read_config(tracker_path)
        robot_config = read_config(robot_path)
        _show_home(robot_config)
        with TrackerCapture(tracker_config["tracker_serials"]) as session:
            _show_devices(session.inspect())
        return 0
    except (KeyboardInterrupt, EOFError):
        print("\nCalibration cancelled.")
        return 130
    except Exception as exc:
        print(f"Tracker calibration: {exc}")
        return 1
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    raise SystemExit(main())
