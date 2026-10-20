# HTC Vive Tracker Wearing Guide

## Getting Started

Before wearing the Trackers, pair them with SteamVR and confirm their serial numbers. After assigning each Tracker to a body segment, write the serial mapping under `openvr.tracker_serials` in `tianji_chest.yaml`. See the [arm configuration template](../src/output_devices/tianji_output/tianji_output/config/tianji_chest.yaml.template).

```yaml
openvr:
  tracker_serials:
    chest: "LHR-xxxxxxx1"            # Chest
    right_hand_back: "LHR-xxxxxxx2"  # Back of right hand
    left_hand_back: "LHR-xxxxxxx3"   # Back of left hand
    left_arm: "LHR-xxxxxxx4"         # Left upper arm
    right_arm: "LHR-xxxxxxx5"        # Right upper arm
```

> [!NOTE]
> SteamVR supports pairing Tracker in headset-free mode by configuring the Null Driver. For setup instructions, see [Configure Null Driver (Headset-Free Mode)](STEAMVR.md). For full usage instructions of wuji-hand-teleop, see [`README.md`](../README.md).

## Wear the Tracker

Secure one Tracker to the back of each hand so it moves with the hand. Keep its mounting position and orientation fixed after calibration. The system uses the raw Tracker origin directly as the hand-back position.

Use the **USB-C port** as the orientation reference for the other Trackers:

- **Arm Tracker**: USB-C port aligned with the arm bone, pointing **toward the torso**
- **Chest Tracker**: USB-C port faces **upward** along the torso

The complete wearing setup is shown below.

![Tracker wearing setup](images/tracker-wearing-combined.jpg)

Configure each robot's flange-to-virtual-hand-back-TCP transform in `driver.tool_transforms.left/right`, then [capture a neutral-pose profile](STEAMVR.md#neutral-pose-calibration) for the configured arm Home. The profile aligns the hand-back Tracker orientation with the TCP axes. Select the exported file with `openvr.calibration_file` in `tianji_chest.yaml` and restart teleoperation. Recalibrate after changing tool geometry, Tracker mounting, or Home.

> [!NOTE]
>
> - Mount the upper arm Tracker on the outer side of the arm, as shown in the figure.
> - Keep the chest Tracker parallel to the torso and secure it to prevent wobbling. Maintain the torso as upright as possible.
> - Ensure all five Trackers remain visible to at least one SteamVR Base Station during operation.
> - When adjusting body posture, keep the relative position between arms and torso unchanged, and move slowly.
