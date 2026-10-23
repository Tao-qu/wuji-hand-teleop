# Continuous recording

The recorder saves LeRobot v2.1 datasets at 30 Hz. Full hands + arms
teleoperation and all five cameras must already be running. Recording owns no
robot connection: it reads robot topics, computes measured palm poses with local
FK, and sends numeric samples to the Jetson. The Jetson aligns camera frames and
writes the dataset locally. Starting, saving, or leaving recording does not
change teleoperation or camera operation.

| Node | Host | Role |
| --- | --- | --- |
| `/data_collection/session` | Desktop | Reads robot topics, computes measured TCP poses, publishes `/data_collection/frame`, and sends recording commands |
| `/data_collection/writer` | Jetson | Subscribes to recording frames and local cameras, serves `/data_collection/command`, and writes episodes |

These interfaces carry recording data and session commands only.

## Setup and start

Build the desktop with `./tools/setup.sh`. Deploy the Jetson packages with
`./tools/deploy_camera.sh --user-deps`, or run `./tools/setup_camera.sh` on the
Jetson. See [camera prerequisites and deployment](CAMERAS.md). Both hosts need
synchronized system clocks, the same `ROS_DOMAIN_ID`, and
`ROS_LOCALHOST_ONLY=0`.

First, start cameras on the Jetson:

```bash
cd ~/Workspace/wuji-hand-teleop
source tools/camera_env.sh
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ros2 run camera_bringup init_gmsl.sh
ros2 launch camera_bringup cameras.launch.py
```

In a second Jetson terminal, start the dataset writer:

```bash
cd ~/Workspace/wuji-hand-teleop
source tools/camera_env.sh
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ros2 run data_collection writer --output ~/Datasets/tianji-wuji2
```

`--output` is a directory on the **Jetson**. Its default is
`~/Datasets/tianji-wuji2`. Keep this process and camera capture running across
episodes. The writer does not start cameras.

Start full teleoperation on the desktop:

```bash
cd ~/Workspace/wuji-hand-teleop
source tools/env.sh
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ros2 run wuji_teleop_bringup wuji_teleop.launch.py
```

Set the task text in `src/data_collection/config/recording.yaml`. Once
teleoperation is ready, open another desktop terminal:

```bash
cd ~/Workspace/wuji-hand-teleop
source tools/env.sh
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ros2 run data_collection record --start 0 --num 10
```

## Configuration and arguments

`src/data_collection/config/recording.yaml` is the local recording configuration.
Setup creates it from the tracked `recording.yaml.template` without overwriting
an existing file:

```yaml
task: ""
start: 0
end: null
num: null
```

| Setting / argument | Meaning | Default |
| --- | --- | --- |
| `task` | Task instruction stored with each saved episode; set a nonempty instruction before recording | Empty |
| `start` / `--start N` | First episode index, starting at zero | `0` |
| `end` / `--end N` | Last episode index, inclusive | `null`, unlimited |
| `num` / `--num N` | Number of episodes to save | `null`, unlimited |
| `--help` | Show command arguments | — |

CLI values override the corresponding configuration values. The effective
`end` and `num` cannot both be set. `start` must be nonnegative, `end` must be at
least `start`, and `num` must be positive. Without `end` or `num`, recording
continues until an episode is saved with **q**.

LeRobot episodes are contiguous: a new dataset starts at 0. If episodes 0–19
already exist, resume with `--start 20`. Existing episodes are not overwritten.
For example:

```bash
ros2 run data_collection record --start 20 --end 29
ros2 run data_collection record --start 30
```

Clear a configured `num` before using `--end`, and clear a configured `end`
before using `--num`.

## Keyboard workflow

| Current state | Key | Result |
| --- | --- | --- |
| Ready | **x** | Start the current episode |
| Recording | **x** | Stop and wait for confirmation |
| Awaiting confirmation | **s** | Save, then prepare the next episode |
| Awaiting confirmation | **r** | Discard; retry the same episode index with **x** |
| Awaiting confirmation | **q** | Save the current episode and exit |

**q** does not exit from Ready or Recording. Only saved episodes increment the
index and count. The final episode still waits for confirmation: **r** retries
it, while **s** saves and exits when `end` or `num` has been reached.
Ctrl-C aborts an unsaved candidate and exits the recording client. A save that
has already started completes before the client exits.

## Recorded fields

The five numeric fields are `float64`:

| Field | Shape | Values |
| --- | --- | --- |
| `action.joint_position` | `[54]` | Arm and hand targets submitted to the SDK, rad |
| `action.ee_pose` | `[14]` | Left and right palm TCP targets before IK |
| `state.joint_position` | `[54]` | Measured arm and hand joints, rad |
| `state.ee_pose` | `[14]` | Palm TCP FK from measured arm joints |
| `state.joint_velocity` | `[54]` | Measured arm and hand joint velocity, rad/s |

Every 54-value joint vector uses the same order:

| Indices | Joint names |
| --- | --- |
| 0–6 | `left_joint_1` … `left_joint_7` |
| 7–13 | `right_joint_1` … `right_joint_7` |
| 14–33 | Left hand, `l_` prefix, suffixes below |
| 34–53 | Right hand, `r_` prefix, suffixes below |

Each hand follows the current SDK mapping, including `finger`:

```text
thumb_cmc_flex, thumb_cmc_abd, thumb_mcp, thumb_ip
index_finger_mcp_flex, index_finger_mcp_abd, index_finger_pip, index_finger_dip
middle_finger_mcp_flex, middle_finger_mcp_abd, middle_finger_pip, middle_finger_dip
ring_finger_mcp_flex, ring_finger_mcp_abd, ring_finger_pip, ring_finger_dip
pinky_mcp_flex, pinky_mcp_abd, pinky_pip, pinky_dip
```

Both end-effector vectors use:

```text
left.x, left.y, left.z, left.qx, left.qy, left.qz, left.qw,
right.x, right.y, right.z, right.qx, right.qy, right.qz, right.qw
```

Position is in metres and orientation is an `x, y, z, w` quaternion. The left
and right poses use their respective `left_base` and `right_base` frames. Action
and state both refer to the robot palm TCP from the Tianji tool configuration.

| Image feature | Source topic | Supplied resolution |
| --- | --- | --- |
| `observation.rgb.head_left` | `/cameras/head_left/image_raw` | 2560 × 1984 |
| `observation.rgb.head_right` | `/cameras/head_right/image_raw` | 2560 × 1984 |
| `observation.rgb.left_wrist` | `/cameras/left_wrist/image_raw` | 2560 × 1984 |
| `observation.rgb.right_wrist` | `/cameras/right_wrist/image_raw` | 2560 × 1984 |
| `observation.rgb.d435` | `/cameras/d435/color/image_raw` | 1280 × 720 |
| `observation.depth.d435` | `/cameras/d435/depth/image_rect_raw` | 1280 × 720 |

RGB streams remain separate at their source resolution and are encoded as
H.264 MP4, 4:2:0, 30 fps, without audio. On the Jetson, the writer automatically
uses GStreamer `nvv4l2h264enc` when its plugins are available, with variable
bitrate `max(1,000,000, width × height × 30 × 0.2)` bits/s. Otherwise it uses
FFmpeg `libx264`, `ultrafast`, CRF 18, `yuv420p`. Both paths use lossy RGB
compression. Depth is a `uint16` two-dimensional array in Parquet,
with lossless Zstandard compression; the ROS depth unit is millimetres and zero
means no depth measurement. Depth is not converted into RGB or video.

The dataset includes the selected samples' source timestamps in integer
nanoseconds, plus the format's relative 30 Hz `timestamp` and indices.
`source_timestamp.action.{left_arm,right_arm,left_hand,right_hand,left_ee,right_ee}`
and `source_timestamp.state.{left_arm,right_arm,left_hand,right_hand}` identify
the robot samples. Image timestamps use `source_timestamp.` followed by the
image feature name above. It does not store receive times or source frame counters. MANUS and Tracker raw data,
torque, current, infrared, and point clouds are not recorded.

The desktop samples the latest robot values at or before each 30 Hz tick.
The writer pairs each tick with the nearest image from each stream within one
frame period. Image pairing runs separately from encoding. Stopping drains the
ticks up to the stop keypress before presenting the save/retry prompt.

## Dataset files

```text
tianji-wuji2/
  meta/
    info.json
    tasks.jsonl
    episodes.jsonl
    episodes_stats.jsonl
    camera_calibration.json
  data/chunk-000/
    episode_000000.parquet
  videos/chunk-000/
    observation.rgb.head_left/episode_000000.mp4
    observation.rgb.head_right/episode_000000.mp4
    observation.rgb.left_wrist/episode_000000.mp4
    observation.rgb.right_wrist/episode_000000.mp4
    observation.rgb.d435/episode_000000.mp4
```

Each episode has one Parquet file and five MP4 files. Parquet contains numeric
fields, depth, timestamps, and the standard frame/episode/task indices. JSON
and JSONL files provide LeRobot v2.1 feature names and shapes, counts, task text,
episode lengths, and statistics. `robot_type` is `tianji-wuji2`.
`meta/camera_calibration.json` stores the available camera intrinsics and
extrinsics from `CameraInfo` and TF
data; unmeasured camera-to-robot transforms are not invented.

The writer preserves the agreed `state.*` and `action.*` field names. A model
training adapter can select and concatenate these fields as needed; dataset
creation does not force them into a policy-specific vector.
