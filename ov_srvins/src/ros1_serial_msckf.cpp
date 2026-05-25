/*
 * Sqrt-VINS: A Sqrt-filter-based Visual-Inertial Navigation System
 * Copyright (C) 2025-2026 Yuxiang Peng
 * Copyright (C) 2025-2026 Chuchu Chen
 * Copyright (C) 2025-2026 Kejian Wu
 * Copyright (C) 2018-2026 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program. If not, see
 * <https://www.gnu.org/licenses/>.
 */





#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <algorithm>
#include <memory>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "ros/ROS1Visualizer.h"
#include "utils/dataset_reader.h"

using namespace ov_srvins;

std::shared_ptr<VioManager> sys;
std::shared_ptr<ROS1Visualizer> viz;

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

  // Launch our ros node
  ros::init(argc, argv, "ros1_serial_msckf");
  auto nh = std::make_shared<ros::NodeHandle>("~");
  nh->param<std::string>("config_path", config_path, config_path);

  // Load the config
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  parser->set_node_handler(nh);

  // Verbosity
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create our VIO system
  VioManagerOptions params;
  params.print_and_load(parser);
  params.num_opencv_threads = 0;       // uncomment if you want repeatability
  params.use_multi_threading_pubs = 0; // uncomment if you want repeatability
  params.use_multi_threading_subs = false;

  sys = std::make_shared<VioManager>(params);
  viz = std::make_shared<ROS1Visualizer>(nh, sys);

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT_ERROR(RED
                "[SERIAL]: unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Our imu topic (single combined) or split gyro+accel topics
  std::string topic_imu;
  nh->param<std::string>("topic_imu", topic_imu, "/imu0");
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);

  std::string topic_gyro, topic_accel;
  nh->param<std::string>("topic_gyro", topic_gyro, "");
  nh->param<std::string>("topic_accel", topic_accel, "");
  bool split_imu = !topic_gyro.empty() && !topic_accel.empty();

  if (split_imu) {
    PRINT_DEBUG("[SERIAL]: split IMU mode — gyro: %s  accel: %s\n",
                topic_gyro.c_str(), topic_accel.c_str());
  } else {
    PRINT_DEBUG("[SERIAL]: imu: %s\n", topic_imu.c_str());
  }

  // Our camera topics
  std::vector<std::string> topic_cameras;
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    std::string cam_topic;
    nh->param<std::string>("topic_camera" + std::to_string(i), cam_topic,
                           "/cam" + std::to_string(i) + "/image_raw");
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(i),
                           "rostopic", cam_topic);
    topic_cameras.emplace_back(cam_topic);
    PRINT_DEBUG("[SERIAL]: cam: %s\n", cam_topic.c_str());
  }

  // Location of the ROS bag we want to read in
  std::string path_to_bag;
  nh->param<std::string>("path_bag", path_to_bag,
                         "/home/patrick/datasets/eth/V1_01_easy.bag");
  PRINT_DEBUG("[SERIAL]: ros bag path is: %s\n", path_to_bag.c_str());

  // Load groundtruth if we have it
  // NOTE: needs to be a csv ASL format file
  std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
  if (nh->hasParam("path_gt")) {
    std::string path_to_gt;
    nh->param<std::string>("path_gt", path_to_gt, "");
    if (!path_to_gt.empty()) {
      ov_core::DatasetReader::load_gt_file(path_to_gt, gt_states);
      PRINT_DEBUG("[SERIAL]: gt file path is: %s\n", path_to_gt.c_str());
    }
  }

  // Get our start location and how much of the bag we want to play
  // Make the bag duration < 0 to just process to the end of the bag
  double bag_start, bag_durr;
  nh->param<double>("bag_start", bag_start, 0);
  nh->param<double>("bag_durr", bag_durr, -1);
  PRINT_DEBUG("[SERIAL]: bag start: %.1f\n", bag_start);
  PRINT_DEBUG("[SERIAL]: bag duration: %.1f\n", bag_durr);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Load rosbag here, and find messages we can play
  rosbag::Bag bag;
  bag.open(path_to_bag, rosbag::bagmode::Read);

  // We should load the bag as a view
  // Here we go from beginning of the bag to the end of the bag
  rosbag::View view_full;
  rosbag::View view;

  // Start a few seconds in from the full view time
  // If we have a negative duration then use the full bag length
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(bag_start);
  ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime()
                                         : time_init + ros::Duration(bag_durr);
  PRINT_DEBUG("time start = %.6f\n", time_init.toSec());
  PRINT_DEBUG("time end   = %.6f\n", time_finish.toSec());
  view.addQuery(bag, time_init, time_finish);

  // Check to make sure we have data to play
  if (view.size() == 0) {
    PRINT_ERROR(
        RED
        "[SERIAL]: No messages to play on specified topics.  Exiting.\n" RESET);
    ros::shutdown();
    return EXIT_FAILURE;
  }

  // We going to loop through and collect a list of all messages
  // This is done so we can access arbitrary points in the bag
  // NOTE: if we instantiate messages here, this requires the whole bag to be
  // read NOTE: thus we just check the topic which allows us to quickly loop
  // through the index NOTE: see this PR
  // https://github.com/ros/ros_comm/issues/117
  // In split-IMU mode, accel messages are buffered separately for interpolation.
  // Gyro messages go into the main msgs vector (time-sorted alongside cameras).
  double max_camera_time = -1;
  std::vector<rosbag::MessageInstance> msgs;
  std::vector<sensor_msgs::Imu::ConstPtr> accel_buf; // only used in split_imu mode

  for (const rosbag::MessageInstance &msg : view) {
    if (!ros::ok())
      break;
    if (split_imu) {
      if (msg.getTopic() == topic_gyro)
        msgs.push_back(msg);
      else if (msg.getTopic() == topic_accel)
        accel_buf.push_back(msg.instantiate<sensor_msgs::Imu>());
    } else {
      if (msg.getTopic() == topic_imu)
        msgs.push_back(msg);
    }
    for (int i = 0; i < params.state_options.num_cameras; i++) {
      if (msg.getTopic() == topic_cameras.at(i)) {
        msgs.push_back(msg);
        max_camera_time = std::max(max_camera_time, msg.getTime().toSec());
      }
    }
  }
  PRINT_INFO("[SERIAL]: total of %zu messages (%zu accel samples buffered)\n",
             msgs.size(), accel_buf.size());

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Forward pointer into accel_buf for O(n) interpolation in split-IMU mode
  size_t accel_idx = 0;

  // Loop through our message array, and lets process them
  std::set<int> used_index;
  for (int m = 0; m < (int)msgs.size(); m++) {

    // End once we reach the last time, or skip if before beginning time
    // (shouldn't happen)
    if (!ros::ok() || msgs.at(m).getTime() > time_finish ||
        msgs.at(m).getTime().toSec() > max_camera_time)
      break;
    if (msgs.at(m).getTime() < time_init)
      continue;

    // Skip messages that we have already used
    if (used_index.find(m) != used_index.end()) {
      used_index.erase(m);
      continue;
    }

    // IMU processing
    const std::string &msg_topic = msgs.at(m).getTopic();
    bool is_imu = split_imu ? (msg_topic == topic_gyro) : (msg_topic == topic_imu);
    if (is_imu) {
      sensor_msgs::Imu::ConstPtr imu_msg;
      if (!split_imu) {
        imu_msg = msgs.at(m).instantiate<sensor_msgs::Imu>();
      } else {
        // Combine gyro with linearly interpolated accel
        auto gyro = msgs.at(m).instantiate<sensor_msgs::Imu>();
        double t = gyro->header.stamp.toSec();

        // Advance accel pointer so accel_buf[accel_idx+1].t >= t (O(n) total)
        while (accel_idx + 1 < accel_buf.size() - 1 &&
               accel_buf[accel_idx + 1]->header.stamp.toSec() < t)
          accel_idx++;

        double ax, ay, az;
        if (accel_idx + 1 < accel_buf.size()) {
          auto &a0 = accel_buf[accel_idx];
          auto &a1 = accel_buf[accel_idx + 1];
          double t0 = a0->header.stamp.toSec();
          double t1 = a1->header.stamp.toSec();
          double alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
          alpha = std::max(0.0, std::min(1.0, alpha));
          ax = a0->linear_acceleration.x + alpha * (a1->linear_acceleration.x - a0->linear_acceleration.x);
          ay = a0->linear_acceleration.y + alpha * (a1->linear_acceleration.y - a0->linear_acceleration.y);
          az = a0->linear_acceleration.z + alpha * (a1->linear_acceleration.z - a0->linear_acceleration.z);
        } else if (!accel_buf.empty()) {
          ax = accel_buf.back()->linear_acceleration.x;
          ay = accel_buf.back()->linear_acceleration.y;
          az = accel_buf.back()->linear_acceleration.z;
        } else {
          continue; // no accel data yet
        }

        auto combined = boost::make_shared<sensor_msgs::Imu>(*gyro);
        combined->header.frame_id = "imu";
        combined->linear_acceleration.x = ax;
        combined->linear_acceleration.y = ay;
        combined->linear_acceleration.z = az;
        combined->orientation_covariance[0] = -1.0;
        imu_msg = combined;
      }
      viz->callback_inertial(imu_msg);
    }

    // Camera processing
    for (int cam_id = 0; cam_id < params.state_options.num_cameras; cam_id++) {

      // Skip if this message is not a camera topic
      if (msg_topic != topic_cameras.at(cam_id))
        continue;

      // We have a matching camera topic here, now find the other cameras for
      // this time For each camera, we will find the nearest timestamp (within
      // 0.02sec) that is greater than the current If we are unable, then this
      // message should just be skipped since it isn't a sync'ed pair!
      std::map<int, int> camid_to_msg_index;
      double meas_time = msgs.at(m).getTime().toSec();
      for (int cam_idt = 0; cam_idt < params.state_options.num_cameras;
           cam_idt++) {
        if (cam_idt == cam_id) {
          camid_to_msg_index.insert({cam_id, m});
          continue;
        }
        int cam_idt_idx = -1;
        for (int mt = m; mt < (int)msgs.size(); mt++) {
          if (msgs.at(mt).getTopic() != topic_cameras.at(cam_idt))
            continue;
          if (std::abs(msgs.at(mt).getTime().toSec() - meas_time) < 0.02)
            cam_idt_idx = mt;
          break;
        }
        if (cam_idt_idx != -1) {
          camid_to_msg_index.insert({cam_idt, cam_idt_idx});
        }
      }

      // Skip processing if we were unable to find any messages
      if ((int)camid_to_msg_index.size() != params.state_options.num_cameras) {
        PRINT_DEBUG(YELLOW "[SERIAL]: Unable to find stereo pair for message "
                           "%d at %.2f into bag (will skip!)\n" RESET,
                    m, meas_time - time_init.toSec());
        continue;
      }

      // Check if we should initialize using the groundtruth
      Eigen::Matrix<double, 17, 1> imustate;
      if (!gt_states.empty() && !sys->initialized() &&
          ov_core::DatasetReader::get_gt_state(meas_time, imustate,
                                               gt_states)) {
        // biases are pretty bad normally, so zero them
        // imustate.block(11,0,6,1).setZero();
        sys->initialize_with_gt(imustate);
      }

      // Relabel 8UC1 → mono8 so cv_bridge can decode D435I infrared images
      auto fix_encoding = [](sensor_msgs::Image::ConstPtr img) -> sensor_msgs::Image::ConstPtr {
        if (img && img->encoding == "8UC1") {
          auto copy = boost::make_shared<sensor_msgs::Image>(*img);
          copy->encoding = "mono8";
          return copy;
        }
        return img;
      };

      // Pass our data into our visualizer callbacks!
      if (params.state_options.num_cameras == 1) {
        viz->callback_monocular(
            fix_encoding(msgs.at(camid_to_msg_index.at(0)).instantiate<sensor_msgs::Image>()),
            0);
      } else if (params.state_options.num_cameras == 2) {
        auto msg0 = msgs.at(camid_to_msg_index.at(0));
        auto msg1 = msgs.at(camid_to_msg_index.at(1));
        used_index.insert(camid_to_msg_index.at(0)); // skip this message
        used_index.insert(camid_to_msg_index.at(1)); // skip this message
        viz->callback_stereo(fix_encoding(msg0.instantiate<sensor_msgs::Image>()),
                             fix_encoding(msg1.instantiate<sensor_msgs::Image>()), 0, 1);
      } else {
        PRINT_ERROR(RED "[SERIAL]: We currently only support 1 or 2 camera "
                        "serial input....\n" RESET);
        return EXIT_FAILURE;
      }

      break;
    }
  }

  // Final visualization
  viz->visualize_final();
  ros::shutdown();

  // Done!
  return EXIT_SUCCESS;
}
