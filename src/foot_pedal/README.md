# USB Foot Pedal

`foot_pedal` reads a PCsensor USB foot pedal and publishes ROS 2 input events.
It operates independently of teleoperation and data collection; it does not
start or stop robot sessions, control recording, or save datasets.

The default device is the keyboard interface of USB device `3553:b001`.
Its Linux key codes map to pedal positions as follows:

| Key | Pedal |
| --- | --- |
| A | Left |
| B | Middle |
| C | Right |

The node reads the selected device directly. Typing A, B, or C on a separate
keyboard does not produce pedal events.

## Installation

Run the project setup from the repository root:

```bash
./tools/setup.sh
source tools/env.sh
```

The default desktop setup installs `evdev==1.9.3`, builds `foot_pedal`, installs
`udev/70-pcsensor-foot-pedal.rules` under `/etc/udev/rules.d`, and reloads udev
rules. Unplug and reconnect the pedal afterward. The rule grants the active local
desktop user access to the matching keyboard interface and creates
`/dev/input/pcsensor-foot-pedal`. SSH or headless sessions require separate device
permissions configured by an administrator.

Use `./tools/setup.sh --skip-system` when system dependencies and device
permissions are already configured. This still installs the Python dependency
and builds the node, but does not install or reload USB rules. The camera setup
profile does not install the pedal package. Setup does not start the node.

## Usage

```bash
source tools/env.sh
ros2 run foot_pedal pedal
```

The node logs connection status and input events. By default, it exclusively
captures the pedal keyboard interface, so pedal presses do not type characters
into the focused application. Other keyboards remain available normally.
Run only one exclusive reader for the pedal at a time.

Press Ctrl-C to stop the node and release the device. After disconnection or an
input queue overflow, the node cancels pending gestures and attempts to reconnect.
A pedal already held down at reconnection must be released before a new press
can produce events.

To select a device explicitly:

```bash
ros2 run foot_pedal pedal --ros-args -p device:=/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd
```

The selected path must still match the configured USB IDs and support A/B/C key
codes. If multiple devices match, automatic selection is refused; specify a
stable device path instead of a numbered `/dev/input/event*` path.

### Device options

Set these parameters at startup:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `device` | Empty | Discover a matching keyboard interface, or use the specified path. |
| `vendor_id` | `13651` (`0x3553`) | USB vendor ID used for device matching. |
| `product_id` | `45057` (`0xb001`) | USB product ID used for device matching. |
| `exclusive` | `true` | Capture pedal input exclusively. When false, pedal keys also reach other applications. |

The long-press threshold is fixed at **2 seconds** and has no user-configurable
parameter.

## ROS 2 topics

With the default node name and namespace:

| Topic | Type | Content |
| --- | --- | --- |
| `/foot_pedal/events` | `std_msgs/msg/String` | One JSON object per input event. |
| `/foot_pedal/status` | `std_msgs/msg/String` | Connection status, published on changes and once per second. |

Events use volatile durability; new subscribers do not receive historical events.
The node also prints events to its log. To view the event topic separately:

```bash
ros2 topic echo /foot_pedal/events
```

Example payload:

```json
{"pedal":"left","key":"A","event":"short","duration_sec":0.25,"sequence":3,"source":"device"}
```

| Field | Meaning |
| --- | --- |
| `pedal` | `left`, `middle`, or `right`. |
| `key` | `A`, `B`, or `C`. |
| `event` | `press`, `release`, `short`, or `long`. |
| `duration_sec` | Hold duration in seconds; zero for `press`. |
| `sequence` | Increasing event counter for the current node process. |
| `source` | `device` for physical pedal input. |

## Press behavior

Each complete pedal gesture produces `press`, then `release`, then exactly one
of `short` or `long`:

- A hold shorter than 2 seconds produces `short`.
- A hold of 2 seconds or longer produces `long`.
- Classification occurs on release, not when the hold reaches 2 seconds.
- Automatic key repeats and duplicate key-down events do not repeat the gesture.

Hold duration uses the original kernel event timestamps. Release and gesture
publication includes an approximately 100 ms confirmation window so a detected
disconnection can cancel synthetic key-up events generated during unplugging.
The confirmation window does not count toward the hold duration. A subsequent
press queued behind a pending release is published in order after that release.

Consumers should choose the appropriate event phase for each action. Mapping
both `press` and `short`/`long` to the same toggle would execute it twice.

The device must report a key held down until the pedal is released. Firmware
configured to send an instantaneous key macro cannot expose physical hold duration.
