# Running sqrtVINS on Intel RealSense D435I Rosbags

Guide for running sqrtVINS on bags recorded with the **RealSense SDK** (not the ROS driver).
The SDK records separate gyro/accel topics and uses non-standard image encodings — the launch
file handles all of that automatically.

---

## Prerequisites

### Docker image

```bash
# Build once (from the sqrtVINS repo root)
docker build --network=host -t sqrtvins_ros1_noetic -f Dockerfile_ros1_20_04 .
```

### Build the workspace

```bash
docker run -it --rm \
  -v /home/yuzedu/sqrt_ws:/catkin_ws \
  -e EIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
  sqrtvins_ros1_noetic bash

# Inside container:
cd /catkin_ws
catkin build -j2 --no-status
exit
```

---

## Running a bag

### Start the container (with RViz display)

```bash
docker run -it --rm \
  --name sqrtvins \
  -v /home/yuzedu/sqrt_ws:/catkin_ws \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e EIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
  sqrtvins_ros1_noetic bash
```

### Inside the container

```bash
source /catkin_ws/devel/setup.bash

roslaunch ov_srvins rs_d435i_rosbag.launch \
  bag:=<path_to_bag> \
  camera:=ir        # ir (default) or rgb
```

| Argument | Default | Description |
|----------|---------|-------------|
| `camera:=ir\|rgb` | `ir` | Camera mode |
| `dorviz:=false` | `true` | Disable RViz |
| `dosave:=true path_est:=<file>` | — | Save pose trajectory (PoseWithCovarianceStamped) |
| `dosave_vel:=true path_body_vel:=<file>` | — | Save body-frame velocity output (see below) |

### Body velocity output

When `dosave_vel:=true`, the estimator writes one line per camera frame to `path_body_vel` with these columns:

```
timestamp px py pz qx qy qz qw vx vy vz wx wy wz num_msckf_points num_slam_points cov_v_00 cov_v_01 cov_v_02 cov_v_11 cov_v_12 cov_v_22
```

- `px py pz` — position in global frame
- `qx qy qz qw` — orientation quaternion (global-to-IMU, JPL, written as x y z w)
- `vx vy vz` — linear velocity in IMU/body frame (post visual update)
- `wx wy wz` — bias-corrected angular velocity in IMU/body frame (from latest gyro)
- `num_msckf_points` — MSCKF features used in the latest update
- `num_slam_points` — active SLAM landmarks in state
- `cov_v_*` — upper triangle of 3x3 velocity covariance in body frame (row-major: 00 01 02 11 12 22)

---

## Serial mode (direct bag reading)

`rs_d435i_serial.launch` reads the bag file directly inside the estimator node — no separate
`rosbag play` or pre-processing scripts needed. This is the recommended way to run on raw D435I
bags because it handles the split gyro/accel streams and 8UC1 image encoding internally, and
produces fully deterministic, repeatable output.

### When to use serial mode vs. rosbag mode

| | Serial (`rs_d435i_serial.launch`) | Rosbag (`rs_d435i_rosbag.launch`) |
|---|---|---|
| Input | Raw D435I bag (split IMU, 8UC1 image) | Pre-processed bag or live ROS topics |
| IMU combining | Done in C++ (linear interp, O(n)) | Done by `imu_combiner.py` |
| Image re-encoding | Done in C++ (8UC1 → mono8) | Done by `image_encoding_fix.py` |
| Repeatability | Fully deterministic | Depends on ROS timing |
| Requires `rosbag play` | No | Yes |

### Running serial mode

```bash
roslaunch ov_srvins rs_d435i_serial.launch \
  bag:=<path_to_bag> \
  camera:=ir          # ir (default) or rgb
```

| Argument | Default | Description |
|----------|---------|-------------|
| `bag` | `rosbags/session1_*.bag` | Path to raw D435I bag |
| `camera:=ir\|rgb` | `ir` | Camera stream to use |
| `bag_start` | `0` | Skip this many seconds from the start |
| `bag_durr` | `-1` | Process this many seconds (-1 = full bag) |
| `topic_gyro` | `/device_0/sensor_2/Gyro_0/imu/data` | Gyro topic in bag |
| `topic_accel` | `/device_0/sensor_2/Accel_0/imu/data` | Accel topic in bag |
| `topic_imu` | `/imu0` | Combined IMU topic (only used if gyro/accel are empty) |
| `topic_camera0` | `/device_0/sensor_0/Infrared_1/image/data` (ir) | Camera topic in bag |
| `dorviz:=true\|false` | `true` | Enable/disable RViz |
| `dosave_vel:=true path_body_vel:=<file>` | — | Save body-frame velocity output |
| `dotime:=true path_time:=<file>` | — | Save per-frame timing statistics |

### How the serial pipeline works

```
rs_d435i_serial.launch
    │
    └─ ros1_serial_msckf (reads bag directly)
            │
            ├─ Gyro  /device_0/sensor_2/Gyro_0/imu/data  ──► combined IMU msg
            ├─ Accel /device_0/sensor_2/Accel_0/imu/data ─┘  (linear interp in C++)
            │
            └─ Image /device_0/sensor_0/Infrared_1/image/data
                     (8UC1 → mono8 relabelled in C++)
                            │
                     sqrtVINS estimator
```

Gyro messages drive the output rate (~200 Hz). For each gyro sample, the nearest two
accelerometer samples are linearly interpolated so the combined IMU message has both
angular velocity and linear acceleration at the same timestamp.

### Serial mode: full launch commands

**Corridor lights-off (69 s)**
```bash
roslaunch ov_srvins rs_d435i_serial.launch \
  bag:="/catkin_ws/src/sqrtVINS/rosbags/Rgb+infrared/corri_lightoff/corri_lightoff_20260523_101011/front_rs_20260523_101011.bag" \
  camera:=ir \
  dorviz:=false \
  dosave_vel:=true \
  path_body_vel:=/catkin_ws/src/sqrtVINS/rosbags/body_velocity.txt
```

**Outdoor grass (70 s)**
```bash
roslaunch ov_srvins rs_d435i_serial.launch \
  bag:="/catkin_ws/src/sqrtVINS/rosbags/Rgb+infrared/grass/grass_20260523_102353/front_rs_20260523_102353.bag" \
  camera:=ir \
  dorviz:=false \
  dosave_vel:=true \
  path_body_vel:=/catkin_ws/src/sqrtVINS/rosbags/body_velocity.txt
```

**Outdoor stones (53 s)**
```bash
roslaunch ov_srvins rs_d435i_serial.launch \
  bag:="/catkin_ws/src/sqrtVINS/rosbags/Rgb+infrared/stones/stones_20260523_102057/front_rs_20260523_102057.bag" \
  camera:=ir \
  dorviz:=false \
  dosave_vel:=true \
  path_body_vel:=/catkin_ws/src/sqrtVINS/rosbags/body_velocity.txt
```

---

## VIO health monitoring

sqrtVINS includes a real-time sliding-window health monitor that runs inside the estimator
at camera rate (post visual update). When the estimator is unhealthy for too many consecutive
frames, it writes `FAIL` to the body velocity output file and stops monitoring.

### Health criteria

A frame is marked **unhealthy** if either condition holds:

| Condition | Threshold | Rationale |
|-----------|-----------|-----------|
| Velocity covariance spike | `cov_v_00 > 0.005` | Filter uncertainty has grown unacceptably |
| Too few tracked features | `num_msckf + num_slam < 5` | Estimator is starved of visual measurements |

### Sliding window logic

```
armed = false   (waits until the estimator is properly initialized)
window = []     (size-20 deque of healthy/unhealthy booleans)

each camera frame:
  if not armed:
    if num_msckf + num_slam >= 5  AND  num_slam > 2:
      armed = true       ← requires SLAM features to be active, not just early init noise

  if armed:
    unhealthy = (cov_v_00 > 0.005) OR (num_msckf + num_slam < 5)
    window.append(unhealthy)
    if len(window) == 20:
      if count(unhealthy in window) >= 16:
        write "FAIL" to body_velocity.txt
        stop monitoring
```

### **Parameters** (compile-time constants in `ROS1Visualizer.h`):

| Constant | Value | Meaning |
|----------|-------|---------|
| `HEALTH_WINDOW_SIZE` | 20 | Rolling window length in frames |
| `HEALTH_FAIL_COUNT` | 16 | Unhealthy frames needed to trigger FAIL |
| `HEALTH_COV_THRESHOLD` | 0.005 | Velocity covariance spike threshold (m²/s²) |
| `HEALTH_FEAT_THRESHOLD` | 5 | Minimum total feature count |

### Reading the output file

`FAIL` appears as a standalone line immediately after the last frame that pushed the window
over the threshold. Frames continue to be written after `FAIL` (the estimator keeps running),
but the health monitor does not write any further diagnostics.

```
# Example snippet from body_velocity.txt
...
1779545430.174 ... 0 0 0.000886 ...   ← unhealthy: 0 features
1779545430.241 ... 0 0 0.001133 ...   ← unhealthy: 0 features
FAIL                                  ← 16/20 window frames were unhealthy
1779545430.308 ... 4 0 0.000978 ...   ← estimator keeps running, monitor stopped
...
```

---

## Available bags

All bags are under `rosbags/Rgb+infrared/`. Every bag contains both IR and RGB streams.

| Bag | Scene | Duration |
|-----|-------|----------|
| `front_rs_20260523_101937.bag` | Indoor front-facing | 65 s |
| `front_rs_20260523_101132-001.bag` | Indoor front-facing (longer) | 94 s |
| `front_rs_20260523_102512-001.bag` | Indoor front-facing (large) | — |
| `corri_lightoff/corri_lightoff_20260523_101011/front_rs_20260523_101011.bag` | Corridor, lights off | 69 s |
| `grass/grass_20260523_102353/front_rs_20260523_102353.bag` | Outdoor grass | 70 s |
| `stones/stones_20260523_102057/front_rs_20260523_102057.bag` | Outdoor stones | 53 s |

### Full launch commands

**Indoor front (65 s)**
```bash
roslaunch ov_srvins rs_d435i_rosbag.launch \
  bag:="/catkin_ws/src/sqrtVINS/rosbags/Rgb+infrared/front_rs_20260523_101937.bag" \
  camera:=ir\
  dosave_vel:=true \
  path_body_vel:=/catkin_ws/src/sqrtVINS/rosbags/body_velocity.txt
```

**Indoor front (94 s)**
```bash
roslaunch ov_srvins rs_d435i_rosbag.launch \
  bag:="/catkin_ws/src/sqrtVINS/rosbags/Rgb+infrared/front_rs_20260523_101132-001.bag" \
  camera:=ir\
  dosave_vel:=true \
  path_body_vel:=/catkin_ws/src/sqrtVINS/rosbags/body_velocity.txt
```

**Corridor lights-off**
```bash
roslaunch ov_srvins rs_d435i_rosbag.launch \
  bag:="/catkin_ws/src/sqrtVINS/rosbags/Rgb+infrared/corri_lightoff/corri_lightoff_20260523_101011/front_rs_20260523_101011.bag" \
  camera:=ir\
  dosave_vel:=true \
  path_body_vel:=/catkin_ws/src/sqrtVINS/rosbags/body_velocity.txt
```

**Outdoor grass**
```bash
roslaunch ov_srvins rs_d435i_rosbag.launch \
  bag:="/catkin_ws/src/sqrtVINS/rosbags/Rgb+infrared/grass/grass_20260523_102353/front_rs_20260523_102353.bag" \
  camera:=ir\
  dosave_vel:=true \
  path_body_vel:=/catkin_ws/src/sqrtVINS/rosbags/body_velocity.txt
```

**Outdoor stones**
```bash
roslaunch ov_srvins rs_d435i_rosbag.launch \
  bag:="/catkin_ws/src/sqrtVINS/rosbags/Rgb+infrared/stones/stones_20260523_102057/front_rs_20260523_102057.bag" \
  camera:=ir \
  dosave_vel:=true \
  path_body_vel:=/catkin_ws/src/sqrtVINS/rosbags/body_velocity.txt
```

---

## Camera mode: IR vs RGB

| | IR (`camera:=ir`) | RGB (`camera:=rgb`) |
|---|---|---|
| Encoding | `8UC1` → relabelled `mono8` | `rgb8` → passed through (cv_bridge converts) |
| Topic | `/device_0/sensor_0/Infrared_1/image/data` | `/device_0/sensor_1/Color_0/image/data` |
| Config | `config/rs_d435i/` | `config/rs_d435i_rgb/` |
| RViz image topic | `/camera/infra1/image_raw` | `/camera/color/image_raw` |
| Outdoor | Good — active IR projector | OK — auto-exposure lag possible |
| Dark/no light | **Required** — IR works in darkness | Unusable |

The `trackhist` visualization always shows grayscale regardless of camera mode.

---

## Config files

| File | Purpose |
|------|---------|
| `config/rs_d435i/estimator_config.yaml` | Main VIO params (IR) |
| `config/rs_d435i/kalibr_imucam_chain.yaml` | IR camera intrinsics + extrinsics |
| `config/rs_d435i/kalibr_imu_chain.yaml` | IMU noise params |
| `config/rs_d435i_rgb/estimator_config.yaml` | Main VIO params (RGB) |
| `config/rs_d435i_rgb/kalibr_imucam_chain.yaml` | RGB camera intrinsics + extrinsics |

All intrinsics and extrinsics are from the factory calibration (`rs-enumerate-devices.txt`).
Online calibration (`calib_cam_extrinsics`, `calib_cam_intrinsics`, `calib_cam_timeoffset`) is
enabled to refine from the factory values during the run.

---

## How the launch pipeline works

```
rosbag play
    │
    ├─ /device_0/sensor_0/Infrared_1/image/data (8UC1)
    │       └─ image_encoding_fix.py ──► /camera/infra1/image_raw (mono8)
    │                                           │
    ├─ /device_0/sensor_1/Color_0/image/data (rgb8)
    │       └─ image_encoding_fix.py ──► /camera/color/image_raw (rgb8, passthrough)
    │                                           │
    ├─ /device_0/sensor_2/Gyro_0/imu/data   ──► imu_combiner.py ──► /imu0
    └─ /device_0/sensor_2/Accel_0/imu/data ─┘
                                                    │
                                             run_subscribe_msckf
                                             (sqrtVINS estimator)
```

The IMU combiner drives output at the gyro rate (~200 Hz) and linearly interpolates
accelerometer readings to match each gyro timestamp.
