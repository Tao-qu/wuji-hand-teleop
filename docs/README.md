# Documentation

- [Project overview, Hand 2 operating modes and explicit enable services](../README.md)
- [Local environment, pinned official SDK/model and build](LOCAL_SETUP.md)
- [Hand 2 migration: official sources, mapping, changed files and acceptance gates](HAND2_MIGRATION.md)
- [Read-only ROS input sampling and JSON summaries](LOCAL_SETUP.md#7-read-only-ros-input-sampling)
- [MANUS calibration, 21-point input contract and validation](../src/input_devices/manus_input/README.md)
- [SteamVR and five Trackers](STEAMVR.md)
- [Tracker wearing guide](tracker-wearing-guide.md)

The target is Wuji Hand 2.2 with the official Wuji SDK. Launch defaults to disabled hand and arm motors; feedback-only hand mode is available separately. Tianji payload values are reused from the cabinet until the new mounted hand assemblies are calibrated. Build and offline-test results do not establish hardware acceptance.

- [Hardware acceptance record — 2026-10-19](HARDWARE_ACCEPTANCE_2026-10-19.md): measured results, failed attempts, corrections, and remaining hardware checks.
- [Attended acceptance runbook](ATTENDED_ACCEPTANCE.md): actuator-free preview, fresh disabled baseline, separate motion stages, and combined acceptance.
- [Right-hand encoder diagnostic](HAND2_ENCODER_DIAGNOSTIC.md): local evidence and manufacturer follow-up questions; no support message has been sent.
