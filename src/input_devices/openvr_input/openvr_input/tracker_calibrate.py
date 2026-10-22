"""Capture anatomical Tracker calibration and pair its neutral pose with robot Home."""

import argparse
import signal
import time

from .tracker_calibration_io import (
    calibration_output_path,
    load_context,
    load_output_profile,
    make_profile,
    save_profile,
)


CAPTURE_SECONDS = 5.0


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def _parser():
    parser = argparse.ArgumentParser(
        description="Calibrate five body-mounted Trackers and review their mapping to robot Home.",
        epilog="Calibration reads OpenVR and local kinematics only. "
               "Select the exported profile with calibration_file in the OpenVR "
               "configuration, then restart teleoperation.",
    )
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--inspect", action="store_true",
                       help="show configured Trackers and Home (default)")
    modes.add_argument("--calibrate", action="store_true",
                       help="capture a neutral pose for 5 seconds and export a calibration")
    parser.add_argument(
        "--output", metavar="FILENAME",
        help="YAML filename in the project openvr_input/calibration directory "
             "(default: tracker.yaml); reuse its offsets if it exists, otherwise use "
             "50 mm per hand; saving replaces the file; paths are not accepted",
    )
    return parser


def _show_devices(devices):
    print("\nConfigured Trackers:")
    for role, device in devices.items():
        print(f"  {role:15s} {device['serial']:16s} {device['status']}")


def _show_home(arm_config, reference):
    print("\nRobot Home reference for neutral-pose retargeting:")
    for side in ("left", "right"):
        position = reference["home_palm"][side][:3, 3].tolist()
        print(f"  {side} joints (degrees): {arm_config[side + '_arm']['home_joints']}")
        print(f"  {side} palm in arm base (metres): {position}")


def _show_offsets(profile):
    print("\nPalm -> Tracker in anatomical palm axes (X fingers, Z hand back; metres):")
    for side in ("left", "right"):
        print(f"  {side}: {profile['hand_back_to_palm'][side]['tracker_offset_m']}")
    print("These measured offsets are preserved; one Home capture cannot estimate them.")


def _show_fit(result):
    fit = result["fit"]
    print("\nMirror-constrained human neutral-pose fit:")
    for side in ("left", "right"):
        values = fit[side]
        print(
            f"  {side}: position {values['mean_position_error_mm']:.3f} mm, "
            f"palm orientation {values['mean_palm_angle_deg']:.3f} deg, "
            f"upper arm {values['mean_upper_angle_deg']:.3f} deg"
        )


def _show_alignment(profile, reference):
    print("\nInitial palm mapping (positions in metres):")
    for side in ("left", "right"):
        neutral = profile["neutral_pose"][side]["position_m"]
        home = reference["home_palm"][side][:3, 3].tolist()
        print(f"  {side}: human chest {neutral} -> robot arm base {home}")
    print("Retargeting maps motion relative to this human neutral pose onto the configured robot Home.")


def _capture_and_fit(context, output):
    from .tracker_capture import TrackerCapture
    from .tracker_calibration import fit_calibration

    input_config = context["input_config"]
    _show_home(context["robot_config"], context["robot_home_reference"])
    _show_offsets(context["profile"])
    with TrackerCapture(input_config["tracker_serials"]) as session:
        _show_devices(session.inspect())
        while True:
            print(
                "\nStand upright with both upper arms naturally spread, elbows bent, "
                "hands in front, palms down, and fingers pointing forward. "
                "Keep the pose symmetric and all five "
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
            _show_alignment(result["calibration"], context["robot_home_reference"])
            while True:
                choice = input("Enter 'save', 'retry', or 'cancel': ").strip().lower()
                if choice in ("save", "retry", "cancel"):
                    break
            if choice == "retry":
                continue
            if choice == "cancel":
                return 2
            profile = make_profile(result)
            path = save_profile(output, profile)
            print(f"Calibration exported to {path}")
            print(f"Set calibration_file in the OpenVR configuration to {path.name}, "
                  "then restart teleoperation.")
            return 0


def main(argv=None):
    parser = _parser()
    args = parser.parse_args(argv)
    if args.output is not None and not args.calibrate:
        parser.error("--output requires --calibrate")
    previous_sigterm = signal.signal(signal.SIGTERM, _interrupt)
    try:
        from .configuration import input_config_path, load_input_config
        from tianji_output.configuration import arm_config_path, load_arm_config
        from tianji_output.kinematics import load_home_reference

        input_path = input_config_path()
        input_config = load_input_config(input_path)
        robot_config = load_arm_config()
        reference = load_home_reference(robot_config)
        output = calibration_output_path(
            "tracker.yaml" if args.output is None else args.output)
        print(f"OpenVR configuration: {input_path}")
        print(f"Robot Home configuration: {arm_config_path()}")
        print(f"Calibration file: {output}")
        if args.calibrate:
            if not output.parent.is_dir():
                raise ValueError(f"Output directory does not exist: {output.parent}")
            context = load_context(input_config, output)
            context.update(robot_config=robot_config, robot_home_reference=reference)
            return _capture_and_fit(context, output)

        from .tracker_capture import TrackerCapture

        profile = load_output_profile(output)
        _show_home(robot_config, reference)
        _show_offsets(profile)
        _show_alignment(profile, reference)
        with TrackerCapture(input_config["tracker_serials"]) as session:
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
