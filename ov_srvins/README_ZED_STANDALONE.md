# Running sqrtVINS on a ZED camera — standalone (no ROS)

`run_zed_msckf` feeds the ZED SDK image + IMU stream **directly** into
`VioManager`, with no ROS in the loop. It replicates the exact unit/format
conventions of `ROS2Visualizer::callback_inertial` / `callback_monocular`.

- **Source:** `src/run_zed_msckf.cpp`
- **CMake target:** `run_zed_msckf` (built when the ZED SDK + CUDA are found —
  both in the no-ROS `build_standalone` branch and in the ROS2/colcon build;
  the ROS2 variant additionally publishes TF + topics, see
  [Visualizing (RViz2)](#visualizing-rviz2--ros2))
- **Output:** prints live pose and writes a TUM-format trajectory file

---

## Prerequisites

| Requirement | Notes |
|---|---|
| ZED SDK | Installed at `/usr/local/zed` (tested with 5.0.7) |
| CUDA | Required by the ZED SDK (tested with 11.4) |
| Eigen3 | `/usr/include/eigen3` |
| OpenCV | 4.x (system) |
| Boost | system / filesystem / thread / date_time |
| A ZED camera | Tested on **ZED 2i** |

> The ZED is an **exclusive USB device**: only one process may open it. Close
> any other ZED program (`ZED_Explorer`, `pyzed` scripts, a ROS wrapper) first.

---

## Build

Built in an **isolated** directory so it does not touch the ROS2 colcon build.

```bash
cd /home/unitree/sqrtvins_ws/src/sqrtVINS/ov_srvins
mkdir -p build_standalone && cd build_standalone

cmake .. \
    -DENABLE_ROS=OFF \
    -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
    -DZED_DIR=/usr/local/zed \
    -DCMAKE_PREFIX_PATH=/usr/local/zed

make run_zed_msckf -j6
```

The binary is produced at:

```
src/sqrtVINS/ov_srvins/build_standalone/run_zed_msckf
```

A successful configure prints `ZED SDK + CUDA found: building run_zed_msckf`.
If you instead see `ZED SDK or CUDA not found, skipping run_zed_msckf`, check
the `ZED_DIR` / `CMAKE_PREFIX_PATH` paths above.

### Rebuild after editing the source

```bash
cd /home/unitree/sqrtvins_ws/src/sqrtVINS/ov_srvins/build_standalone
make run_zed_msckf -j6
```

### ROS2 variant (publishes TF/topics, see "Visualizing (RViz2)")

```bash
source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash
cd ~/sqrtvins_ws
colcon build --packages-select ov_srvins
```

Installs to `install/ov_srvins/lib/ov_srvins/run_zed_msckf`
(run via `ros2 run ov_srvins run_zed_msckf ...`). The two builds coexist —
`build_standalone` is not touched by colcon.

---

## Run

```bash
cd /home/unitree/sqrtvins_ws

./src/sqrtVINS/ov_srvins/build_standalone/run_zed_msckf \
    src/sqrtVINS/config/zed_imu/estimator_config.yaml \
    zed_traj_tum.txt
```

| Argument | Meaning | Default |
|---|---|---|
| 1 | path to `estimator_config.yaml` | `src/sqrtVINS/config/zed_imu/estimator_config.yaml` |
| 2 | output TUM trajectory file | `zed_traj_tum.txt` |

Press **Ctrl-C** (SIGINT) or `kill` (SIGTERM) to stop cleanly.

### Expected startup output

```
[zed] opened ZED 2i SN 35692113
[init]: successful initialization in 0.0098 seconds
[zed] t=...  p=[ 0.000  0.000  0.000]
...
```

Pose stays near zero while the camera is stationary (the zero-velocity update
pins it). Move the camera and the position/orientation will track.

---

## Output format

**Trajectory** — TUM format, one line per processed frame (`<arg2>`, default
`zed_traj_tum.txt`):

```
timestamp  tx ty tz  qx qy qz qw
```

- Pose is the IMU state in the global frame (`state->imu`).
- The quaternion is **JPL** `[x, y, z, w]` (OpenVINS convention). Convert to
  Hamilton if your downstream tool (e.g. `evo`) expects it.

**Feature cloud** — `<trajectory>.feat` (e.g. `zed_traj_tum.txt.feat`),
**overwritten every frame** with the current 3D features in the global frame:

```
m  x y z      # MSCKF feature (used in the last update)
s  x y z      # active SLAM landmark
```

`m` rows come from `VioManager::get_good_features_MSCKF()`, `s` rows from
`get_features_SLAM()`. The console also prints the per-frame counts
(`MSCKF=.. SLAM=..`).

---

## Visualizing (OpenCV)

`zed_traj_view.py` (workspace root) draws a top-down map with the trajectory
and both feature clouds overlaid. Needs the python that has OpenCV
(**python3.8**) and an X display.

```bash
# static (render a finished run)
python3.8 zed_traj_view.py zed_traj_tum.txt

# live (run in a 2nd terminal while run_zed_msckf writes the file)
python3.8 zed_traj_view.py zed_traj_tum.txt --live
```

- Blue line = path, **green dot** = start, **red dot** = current pose.
- **Yellow dots** = MSCKF features, **magenta dots** = SLAM landmarks
  (auto-loaded from `<traj>.feat`; the view auto-fits to include them).
- Axes: red = X-forward (up), green = Y-left. 1 m grid.
- Keys: `q`/`ESC` quit, `f` refit (handy in `--live`).

---

## Visualizing (RViz2 / ROS2)

The ROS2/colcon build of `run_zed_msckf` publishes while it writes the TUM
file (same CLI, same file output):

| What | Name | Type |
|---|---|---|
| VINS pose (moving frame) | TF `vins_world → zed_imu` | tf2 |
| World alignment (identity) | TF `odom → vins_world` | tf2 static |
| Trajectory | `/vins_path` | `nav_msgs/Path` |
| Current MSCKF + SLAM features | `/vins_points` | `sensor_msgs/PointCloud2` |

> **conda:** deactivate any conda env first (`conda deactivate`) — ROS2 Foxy's
> compiled Python extensions are built for python3.8 and the build/run tooling
> breaks under a conda python.

**Terminal 1 — sport-mode odometry bridge** (publishes TF
`odom → sportmode_base` + `/sportmode_path` from the Go2's
`/odommodestate` SportModeState, for side-by-side comparison):

```bash
source ~/sqrtvins_ws/install/setup.bash   # chains /opt/ros/foxy + ~/ros2_ws
python3 ~/sqrtvins_ws/sportmode_to_odom.py
```

**Terminal 2 — VINS runner (ROS2 build):**

```bash
source ~/sqrtvins_ws/install/setup.bash
cd ~/sqrtvins_ws
ros2 run ov_srvins run_zed_msckf \
    src/sqrtVINS/config/zed_imu/estimator_config.yaml \
    zed_traj_tum.txt
```

Optional — pin the ZED's physical mounting point onto the robot body
(edit the numbers to the real offset):

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 sportmode_base zed_imu_mount
```

The resulting TF tree:

```
odom ─→ sportmode_base ─→ zed_imu_mount   (robot's sport-mode odometry)
  └──→ vins_world ─→ zed_imu             (VINS estimate)
```

**RViz2 setup** (`rviz2` on the robot's display / `ssh -X`, or on a laptop on
the same network + `ROS_DOMAIN_ID` — all topics are standard types, no custom
packages needed):

1. **Global Options → Fixed Frame** = `odom`.
2. **Add → TF** — shows `sportmode_base`, `zed_imu`, … as moving axes
   (enable *Show Names*).
3. **Add → By topic → /vins_path → Path** — the VINS trajectory.
4. **Add → By topic → /sportmode_path → Path** — the sport-mode trajectory
   (pick a different color to tell them apart).
5. **Add → By topic → /vins_points → PointCloud2** — set *Size* ≈ 0.03 m and
   *Color Transformer* = AxisColor (points are xyz-only, no RGB channel).

The gap between the `zed_imu` and `zed_imu_mount` frames is directly the
disagreement between the VINS and sport-mode estimators.

Alternative without the ROS2 binary: `~/sqrtvins_ws/vins_traj_to_tf.py` tails
a TUM file and publishes the same frames/path (`--run <config>` also launches
the standalone binary). Don't run it together with the ROS2 binary — two
writers for the same TF frames.

---

## How it works

Two threads share the open `sl::Camera`:

- **IMU thread** — tight-polls `getSensorsData(CURRENT)` at the full ~400 Hz,
  dedups on the hardware timestamp, converts gyro **deg/s → rad/s** (the SDK
  reports deg/s; OpenVINS wants rad/s), accel is already m/s², then calls
  `feed_measurement_imu`.
- **Camera thread** — `grab()` → `retrieveImage(VIEW::LEFT)` (rectified) →
  BGRA→MONO8 → throttle to `track_frequency` → queue → feed every image
  bracketed by the latest IMU time into `feed_measurement_camera`. After each
  update it prints the pose, appends to the TUM file, and writes the current
  MSCKF + SLAM feature cloud to `<trajectory>.feat`.

Image is captured at **VGA 672×376** to match the calibration in
`config/zed_imu/kalibr_imucam_chain.yaml`. IMU/camera timestamps come from the
same ZED SDK clock, so no cross-clock alignment is needed.

> **Coordinate system (critical).** The runner sets
> `init.coordinate_system = RIGHT_HANDED_Z_UP_X_FWD` (ROS / REP-103 convention).
> The SDK *rotates the IMU accel/gyro axes* to match this. `T_cam_imu` in
> `kalibr_imucam_chain.yaml` was calibrated in this frame (X-fwd, Y-left, Z-up,
> as the ZED ROS2 wrapper publishes `imu/data_raw`). Using the SDK default
> `IMAGE` frame instead makes the IMU and camera disagree — the filter looks
> fine at rest (ZUPT pins it) but **diverges as soon as the camera moves**.

---

## Configuration & accuracy

The calibration files under `config/zed_imu/` are the critical dependency:

- `kalibr_imucam_chain.yaml` — camera intrinsics `[fx, fy, cx, cy] = [266, 266, 339, 190]`,
  resolution `[672, 376]`, IMU→camera extrinsic `T_cam_imu`, and
  `timeshift_cam_imu`.
- `kalibr_imu_chain.yaml` — IMU noise densities / random walks and intrinsics.

If tracking accuracy is poor, check these first — the extrinsic and the
time offset dominate VIO quality. If you change the capture resolution in
`run_zed_msckf.cpp`, you **must** re-supply matching intrinsics.

Currently **monocular** (`max_cameras: 1`, `use_stereo: false`). Stereo would
require grabbing `VIEW::RIGHT` as well and adding `cam1` extrinsics to the
config.

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `camera open failed (busy?)` | Another process holds the ZED — close it. |
| `failed to parse all parameters` | Wrong config path, or a relative include (`kalibr_*`) can't be found — run from the workspace root or pass an absolute config path. |
| `skipping run_zed_msckf` at cmake time | ZED SDK / CUDA not located — fix `ZED_DIR` / `CMAKE_PREFIX_PATH`. |
| Pose never leaves zero | Camera is stationary (ZUPT) — move it; needs texture + motion to initialize fully. |
| Pose fine at rest but **diverges/explodes when moved** | IMU axis convention mismatch — ensure `init.coordinate_system = RIGHT_HANDED_Z_UP_X_FWD` (matches the kalibr calibration). See "Coordinate system" above. |
| `nan` in the trajectory | Was a torn write on hard kill before the SIGTERM handler was added; rebuild the current source. |
