# Changelog

## Current release

- Two MANUS gloves and five HTC Vive Trackers control two Wuji Hand 2.2 hands and two Tianji arms.
- Teleoperation supports combined, hands-only, and arms-only modes; Home returns the selected devices to their configured poses.
- RViz Preview displays finger targets and Tracker poses without actuator drivers.
- The Qt monitor provides mode selection, joint feedback, currents, command rates, hand serial configuration, and a separate arm brake tool.
- Interactive MANUS and Tracker calibration tools write profiles used by the shared runtime configuration.
- Ubuntu 22.04 and ROS 2 Humble setup uses pinned dependencies, a Python environment, a symlink build, and environment checks.
- Teleoperation and Home share a Tianji hardware driver. Tracker retargeting publishes typed palm poses and upper-arm directions; local IK converts these into named joint targets independently of the input device. Robot joint interfaces use radians and expose successfully submitted SDK targets separately from measured feedback.
- Robot palm TCP geometry and human Tracker mounting offsets are configured separately. Tracker calibration uses the measured mounting offsets when fitting the Home reference.
- Jetson camera capture provides four independent GMSL RGB streams and official RealSense D435 color/depth streams, with separate camera configuration, build, and source deployment tools.
