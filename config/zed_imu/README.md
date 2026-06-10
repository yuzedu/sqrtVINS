# sqrtVINS — ZED IMU Config

Uses the ZED camera's built-in IMU (`/h_camera/zed_node/imu/data`) and RGB image
(`/h_camera/zed_node/rgb/image_rect_color`). Both are hardware-synchronized by the ZED SDK,
so no timestamp offset correction is needed.

## Launch

**Terminal 1 — estimator + rviz (inside Docker):**
```bash
ros2 launch ov_srvins subscribe_zed_imu.launch.py rviz_enable:=true
```

**Terminal 2 — play bag (inside Docker):**
```bash
ros2 bag play rosbag2_2026_06_10-11_51_47/rosbag2_2026_06_10-11_51_47
```

## Config files

| File | Purpose |
|------|---------|
| `estimator_config.yaml` | Main sqrtVINS parameters, static initializer (`init_dyn_use: false`) |
| `kalibr_imucam_chain.yaml` | Camera intrinsics + T_cam_imu extrinsic (ZED cam → ZED IMU) |
| `kalibr_imu_chain.yaml` | ZED IMU noise parameters, topic `/h_camera/zed_node/imu/data` |

## Calibration source

iKalibr 2.0 output (`ikalibr_zed.txt`). ZED IMU is the reference frame.
Camera-IMU translation is ~8.5 cm (physically consistent with ZED hardware).
