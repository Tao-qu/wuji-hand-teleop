"""Interactive five-Tracker calibration for the configured Tianji arm Home."""

import argparse
from pathlib import Path
import signal
import time

from .tracker_calibration_io import (
    default_config,
    load_context,
    make_profile,
    read_config,
    save_profile,
)


CAPTURE_SECONDS = 5.0


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def _parser():
    parser = argparse.ArgumentParser(
        description="Inspect or calibrate five Trackers against the configured arm Home.",
        epilog="Calibration reads OpenVR and local kinematics only. "
               "Copy exported calibration fields into the project configuration manually, "
               "then restart teleoperation.",
    )
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--inspect", action="store_true",
                       help="show configured Trackers and Home (default)")
    modes.add_argument("--calibrate", action="store_true",
                       help="capture a neutral pose for 5 seconds and export a calibration")
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


def _capture_and_fit(context, output):
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
            print(f"Capture started. Hold the pose for {CAPTURE_SECONDS:g} seconds.", flush=True)
            samples, stats = session.capture(duration=CAPTURE_SECONDS)
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
            path = save_profile(output, profile)
            print(f"Calibration exported to {path}")
            print("Copy calibration.chest_mount and calibration.wrist_to_tianji into "
                  "wuji_teleop_bringup/config/static_transforms.yaml.")
            print("Copy calibration.upper_arm_rotations into "
                  "openvr_input/config/openvr_input.yaml, then restart teleoperation.")
            return 0


def main(argv=None):
    parser = _parser()
    args = parser.parse_args(argv)
    if args.calibrate and args.output is None:
        parser.error("--output is required with --calibrate")
    if args.output is not None and not args.calibrate:
        parser.error("--output requires --calibrate")
    if args.output is not None:
        args.output = args.output.expanduser()
    previous_sigterm = signal.signal(signal.SIGTERM, _interrupt)
    try:
        tracker_path = default_config("openvr_input", "openvr_input.yaml")
        robot_path = default_config("tianji_output", "tianji_chest.yaml")
        if args.calibrate:
            if args.output.exists() or args.output.is_symlink():
                raise ValueError(f"Choose a new output file: {args.output}")
            if not args.output.parent.is_dir():
                raise ValueError(f"Output directory does not exist: {args.output.parent}")
            static_path = default_config("wuji_teleop_bringup", "static_transforms.yaml")
            kinematics_path = default_config("tianji_output", "ccs_m6.MvKDCfg")
            context = load_context(tracker_path, static_path, robot_path, kinematics_path)
            return _capture_and_fit(context, args.output)

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
