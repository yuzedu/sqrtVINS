/*
 * Sqrt-VINS standalone live runner for the ZED stereo camera.
 *
 * Feeds the ZED SDK image (left, rectified) + IMU directly into VioManager,
 * mirroring the unit/format conventions of ROS2Visualizer::callback_inertial
 * and callback_monocular. Builds with -DENABLE_ROS=OFF (no ROS, file output
 * only) or under the ROS2/colcon build (ROS_AVAILABLE=2), which additionally
 * publishes TF vins_world->zed_imu (odom->vins_world static), /vins_path
 * (nav_msgs/Path), and /vins_points (sensor_msgs/PointCloud2 of the current
 * MSCKF+SLAM features).
 *
 *   ./run_zed_msckf <path-to-estimator_config.yaml> [output_traj.txt]
 *
 * Output: prints pose + writes a TUM-format trajectory file
 *   timestamp tx ty tz qx qy qz qw   (JPL quaternion, as stored by OpenVINS)
 */
#include <atomic>
#include <csignal>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "types/IMU.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/sensor_data.h"

#if ROS_AVAILABLE == 2
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#endif

using namespace ov_srvins;

static std::atomic<bool> g_stop{false};
static void on_sigint(int) { g_stop = true; }

constexpr double DEG2RAD = 0.01745329251994329577;

int main(int argc, char **argv) {
#if ROS_AVAILABLE == 2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("run_zed_msckf");
  tf2_ros::TransformBroadcaster tf_br(node);
  tf2_ros::StaticTransformBroadcaster static_br(node);
  auto path_pub = node->create_publisher<nav_msgs::msg::Path>("/vins_path", 10);
  auto cloud_pub =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("/vins_points", 1);
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "vins_world";

  // Pin the VINS origin into the odom tree (identity: both estimators are
  // assumed zeroed at the same pose; adjust here if not).
  geometry_msgs::msg::TransformStamped align;
  align.header.stamp = node->now();
  align.header.frame_id = "odom";
  align.child_frame_id = "vins_world";
  align.transform.rotation.w = 1.0;
  static_br.sendTransform(align);
#endif

  // Installed after rclcpp::init so these replace rclcpp's SIGINT handler
  // and g_stop is still set on Ctrl+C (we shut rclcpp down ourselves).
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  std::string config_path =
      (argc > 1) ? argv[1]
                 : "src/sqrtVINS/config/zed_imu/estimator_config.yaml";
  std::string traj_path = (argc > 2) ? argv[2] : "zed_traj_tum.txt";

  // ---- Load config (file-based, no ROS) ----
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = true;
  if (!parser->successful()) {
    std::cerr << "[zed] failed to parse all parameters in " << config_path
              << std::endl;
    return EXIT_FAILURE;
  }
  auto sys = std::make_shared<VioManager>(params);

  // ---- Open the ZED ----
  sl::Camera zed;
  sl::InitParameters init;
  init.camera_resolution = sl::RESOLUTION::VGA; // 672x376, matches calibration
  init.camera_fps = 100;
  init.coordinate_units = sl::UNIT::METER;
  // IMU axes MUST match the kalibr calibration. T_cam_imu in
  // kalibr_imucam_chain.yaml is in ROS convention (X-fwd, Y-left, Z-up), the
  // frame the ZED ROS2 wrapper publishes imu/data_raw in. The SDK default is
  // IMAGE (Y-down, Z-fwd); using it makes the filter diverge under motion.
  init.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
  init.depth_mode = sl::DEPTH_MODE::NONE; // VINS does its own tracking
  if (zed.open(init) != sl::ERROR_CODE::SUCCESS) {
    std::cerr << "[zed] camera open failed (busy?)" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "[zed] opened " << zed.getCameraInformation().camera_model
            << " SN " << zed.getCameraInformation().serial_number << std::endl;

  std::ofstream traj(traj_path);
  traj << std::fixed << std::setprecision(9);
  // Current feature point-cloud snapshot (overwritten each frame), for the
  // viewer: lines "m x y z" (MSCKF) and "s x y z" (SLAM), global frame.
  std::string feat_path = traj_path + ".feat";

  // ---- Shared state between threads ----
  std::deque<ov_core::CameraData> camera_queue;
  std::mutex queue_mtx;
  std::atomic<double> latest_imu_t{-1.0};

  // ---- IMU thread: tight poll, dedup, feed (never blocked by updates) ----
  std::thread imu_thread([&]() {
    sl::SensorsData sd;
    uint64_t last_ts = 0;
    while (!g_stop) {
      if (zed.getSensorsData(sd, sl::TIME_REFERENCE::CURRENT) !=
          sl::ERROR_CODE::SUCCESS)
        continue;
      uint64_t ts = sd.imu.timestamp.getNanoseconds();
      if (ts == last_ts)
        continue;
      last_ts = ts;

      ov_core::ImuData m;
      m.timestamp = ts * 1e-9;
      // SDK gyro is deg/s -> OpenVINS wants rad/s; accel already m/s^2.
      m.wm << sd.imu.angular_velocity[0] * DEG2RAD,
          sd.imu.angular_velocity[1] * DEG2RAD,
          sd.imu.angular_velocity[2] * DEG2RAD;
      m.am << sd.imu.linear_acceleration[0], sd.imu.linear_acceleration[1],
          sd.imu.linear_acceleration[2];
      sys->feed_measurement_imu(m);
      latest_imu_t = m.timestamp;
    }
  });

  // ---- Camera thread: grab, push, then process queued images ----
  std::thread cam_thread([&]() {
    sl::Mat left;
    sl::RuntimeParameters rt;
    double last_cam_t = -1.0;
    const double min_dt = 1.0 / params.track_frequency;

    while (!g_stop) {
      if (zed.grab(rt) != sl::ERROR_CODE::SUCCESS)
        continue;
      zed.retrieveImage(left, sl::VIEW::LEFT);
      double t =
          left.timestamp.getNanoseconds() * 1e-9; // same clock as IMU

      // Throttle to track_frequency (matches callback_monocular).
      if (last_cam_t > 0 && t < last_cam_t + min_dt)
        continue;
      last_cam_t = t;

      // BGRA (sl::Mat) -> MONO8 cv::Mat, deep-copied off the SDK buffer.
      cv::Mat bgra(left.getHeight(), left.getWidth(), CV_8UC4,
                   left.getPtr<sl::uchar1>(sl::MEM::CPU), left.getStepBytes());
      cv::Mat gray;
      cv::cvtColor(bgra, gray, cv::COLOR_BGRA2GRAY);

      ov_core::CameraData cam;
      cam.timestamp = t;
      cam.sensor_ids.push_back(0);
      cam.images.push_back(gray.clone());
      cam.masks.push_back(cv::Mat::zeros(gray.rows, gray.cols, CV_8UC1));
      {
        std::lock_guard<std::mutex> lck(queue_mtx);
        camera_queue.push_back(cam);
        std::sort(camera_queue.begin(), camera_queue.end());
      }

      // Process any image bracketed by IMU (mirrors callback_inertial).
      double imu_t = latest_imu_t.load();
      if (imu_t < 0)
        continue;
      double t_imu_inC =
          imu_t - sys->get_state()->calib_dt_CAMtoIMU->value()(0);
      std::lock_guard<std::mutex> lck(queue_mtx);
      while (!camera_queue.empty() &&
             camera_queue.front().timestamp < t_imu_inC) {
        sys->feed_measurement_camera(camera_queue.front());
        camera_queue.pop_front();

        if (sys->initialized()) {
          auto st = sys->get_state();
          auto p = st->imu->pos();
          auto q = st->imu->quat(); // JPL [x,y,z,w]

          // Current feature clouds (global frame): MSCKF (used in last update)
          // and active SLAM landmarks.
          auto feats_msckf = sys->get_good_features_MSCKF();
          auto feats_slam = sys->get_features_SLAM();
          printf("[zed] t=%.3f  p=[% .3f % .3f % .3f]  MSCKF=%zu  SLAM=%zu\n",
                 st->timestamp, (double)p(0), (double)p(1), (double)p(2),
                 feats_msckf.size(), feats_slam.size());

          traj << st->timestamp << " " << (double)p(0) << " " << (double)p(1)
               << " " << (double)p(2) << " " << (double)q(0) << " "
               << (double)q(1) << " " << (double)q(2) << " " << (double)q(3)
               << "\n";
          traj.flush();

          std::ofstream feat(feat_path, std::ios::trunc);
          feat << std::fixed << std::setprecision(4);
          for (const auto &f : feats_msckf)
            feat << "m " << (double)f(0) << " " << (double)f(1) << " "
                 << (double)f(2) << "\n";
          for (const auto &f : feats_slam)
            feat << "s " << (double)f(0) << " " << (double)f(1) << " "
                 << (double)f(2) << "\n";

#if ROS_AVAILABLE == 2
          // Stamp with wall clock (st->timestamp is ZED-relative seconds and
          // would be rejected as stale by tf2/RViz).
          auto now = node->now();

          // JPL q_GtoI components read as Hamilton give R_ItoG, which is
          // exactly what TF expects -- copy through unswapped.
          geometry_msgs::msg::TransformStamped tf;
          tf.header.stamp = now;
          tf.header.frame_id = "vins_world";
          tf.child_frame_id = "zed_imu";
          tf.transform.translation.x = (double)p(0);
          tf.transform.translation.y = (double)p(1);
          tf.transform.translation.z = (double)p(2);
          tf.transform.rotation.x = (double)q(0);
          tf.transform.rotation.y = (double)q(1);
          tf.transform.rotation.z = (double)q(2);
          tf.transform.rotation.w = (double)q(3);
          tf_br.sendTransform(tf);

          geometry_msgs::msg::PoseStamped ps;
          ps.header.stamp = now;
          ps.header.frame_id = "vins_world";
          ps.pose.position.x = (double)p(0);
          ps.pose.position.y = (double)p(1);
          ps.pose.position.z = (double)p(2);
          ps.pose.orientation.x = (double)q(0);
          ps.pose.orientation.y = (double)q(1);
          ps.pose.orientation.z = (double)q(2);
          ps.pose.orientation.w = (double)q(3);
          path_msg.header.stamp = now;
          path_msg.poses.push_back(ps);
          if (path_msg.poses.size() > 5000)
            path_msg.poses.erase(path_msg.poses.begin());
          path_pub->publish(path_msg);

          sensor_msgs::msg::PointCloud2 cloud;
          cloud.header.stamp = now;
          cloud.header.frame_id = "vins_world";
          sensor_msgs::PointCloud2Modifier mod(cloud);
          mod.setPointCloud2FieldsByString(1, "xyz");
          mod.resize(feats_msckf.size() + feats_slam.size());
          sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
          sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
          sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
          for (const auto &f : feats_msckf) {
            *it_x = (float)f(0);
            *it_y = (float)f(1);
            *it_z = (float)f(2);
            ++it_x, ++it_y, ++it_z;
          }
          for (const auto &f : feats_slam) {
            *it_x = (float)f(0);
            *it_y = (float)f(1);
            *it_z = (float)f(2);
            ++it_x, ++it_y, ++it_z;
          }
          cloud_pub->publish(cloud);
#endif
        }
      }
    }
  });

  imu_thread.join();
  cam_thread.join();
  zed.close();
  traj.close();
  std::cout << "[zed] stopped. trajectory -> " << traj_path << std::endl;
#if ROS_AVAILABLE == 2
  rclcpp::shutdown();
#endif
  return EXIT_SUCCESS;
}
