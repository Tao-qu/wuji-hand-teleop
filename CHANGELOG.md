# Changelog

## Current release

- Two MANUS gloves and five HTC Vive Trackers control two Wuji Hand 2.2 hands and two Tianji arms.
- Teleoperation supports combined, hands-only, and arms-only modes; Home returns the selected devices to their configured poses.
- RViz Preview displays finger targets and Tracker poses without actuator drivers.
- The Qt monitor provides mode selection, joint feedback, currents, command rates, hand serial configuration, and a separate arm brake tool.
- Interactive MANUS and Tracker calibration tools write profiles used by the shared runtime configuration.
- Ubuntu 22.04 and ROS 2 Humble setup uses pinned dependencies, a Python environment, a symlink build, and environment checks.
- Teleoperation and Home share a Tianji hardware driver. Tracker retargeting publishes typed palm poses and upper-arm directions; local IK converts these into named joint targets independently of the input device. Robot joint interfaces use radians and expose successfully submitted SDK targets separately from measured feedback.
- OpenVR input has its own settings and anatomical human calibration profile. The calibration CLI displays robot Home from local forward kinematics while fitting human mounting rotations and saving a neutral pose. Retargeting maps neutral-relative motion onto configured robot Home using per-arm body-to-base rotations.
- Robot palm TCP geometry and human Tracker mounting offsets are configured separately. Robot Home or tool changes recompute the runtime mapping on restart; Tracker mounting changes require input recalibration.
- MANUS-to-MediaPipe conversion belongs to the controller, and Preview uses the same configured coordinate reflection as live retargeting.
- Jetson camera capture provides four independent GMSL RGB streams and official RealSense D435 color/depth streams, with separate camera configuration, build, and source deployment tools.
