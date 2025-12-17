//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "include/livox_ros_driver.h"

#include <chrono>
#include <vector>
#include <csignal>
#include <ctime>
#include <time.h>
#include <sys/stat.h>

#include <ros/ros.h>
#include "lddc.h"
#include "lds_lidar.h"
#include "lds_lvx.h"
#include "livox_sdk.h"

using namespace livox_ros;

const int32_t kSdkVersionMajorLimit = 2;
Lddc *g_lddc = nullptr;

inline void SignalHandler(int signum) {
  printf("\n=======================\n");
  printf("Signal %d received, shutting down...\n", signum);

  if (g_lddc) {
    printf("Closing bag file...\n");
    g_lddc->PrepareExit();
    printf("Bag file closed.\n");
  }

  ros::shutdown();
  printf("Shutdown complete.\n");
  printf("=======================\n\n");
  exit(signum);
}

int main(int argc, char **argv) {
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "livox_lidar_publisher");
  ros::NodeHandle livox_node;

  ROS_INFO("Livox Ros Driver Version: %s", LIVOX_ROS_DRIVER_VERSION_STRING);
  signal(SIGINT, SignalHandler);
  /** Check sdk version */
  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  if (_sdkversion.major < kSdkVersionMajorLimit) {
    ROS_INFO("The SDK version[%d.%d.%d] is too low", _sdkversion.major,
             _sdkversion.minor, _sdkversion.patch);
    return 0;
  }

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag   = false;

  livox_node.getParam("xfer_format", xfer_format);
  livox_node.getParam("multi_topic", multi_topic);
  livox_node.getParam("data_src", data_src);
  livox_node.getParam("publish_freq", publish_freq);
  livox_node.getParam("output_data_type", output_type);
  livox_node.getParam("frame_id", frame_id);
  livox_node.getParam("enable_lidar_bag", lidar_bag);
  livox_node.getParam("enable_imu_bag", imu_bag);
  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.1) {
    publish_freq = 0.1;
  } else {
    publish_freq = publish_freq;
  }

  /** Lidar data distribute control and lidar data source set */
  Lddc *lddc = new Lddc(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, lidar_bag, imu_bag);
  lddc->SetRosNode(&livox_node);
  g_lddc = lddc;

  int ret = 0;
  if (data_src == kSourceRawLidar) {
    ROS_INFO("Data Source is raw lidar.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    ROS_INFO("Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    livox_node.getParam("cmdline_str", cmdline_bd_code);

    // 创建 bag 文件名（基于当前时间，在 bag_files 目录）
    if (output_type == kOutputToRosBagFile) {
      // 创建 bag_files 目录
      std::string bag_dir = "/home/pi/ws_livox_nopcl/bag_files";
      mkdir(bag_dir.c_str(), 0755);

      time_t now = time(nullptr);
      struct tm *timeinfo = localtime(&now);
      char timestamp[64];
      strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", timeinfo);
      std::string bag_file_path = bag_dir + "/livox_" + std::string(timestamp) + ".bag";
      lddc->CreateBagFile(bag_file_path);
      ROS_INFO("Recording to bag file: %s", bag_file_path.c_str());
    }

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

    LdsLidar *read_lidar = LdsLidar::GetInstance(1000 / publish_freq);
    lddc->RegisterLds(static_cast<Lds *>(read_lidar));
    ret = read_lidar->InitLdsLidar(bd_code_list, user_config_path.c_str());
    if (!ret) {
      ROS_INFO("Init lds lidar success!");
    } else {
      ROS_ERROR("Init lds lidar fail!");
    }
  } else {
    ROS_INFO("Data Source is lvx file.");

    std::string cmdline_file_path;
    livox_node.getParam("cmdline_file_path", cmdline_file_path);

    do {
      if (!IsFilePathValid(cmdline_file_path.c_str())) {
        ROS_ERROR("File path invalid : %s !", cmdline_file_path.c_str());
        break;
      }

      std::string rosbag_file_path;
      int path_end_pos = cmdline_file_path.find_last_of('.');
      rosbag_file_path = cmdline_file_path.substr(0, path_end_pos);
      rosbag_file_path += ".bag";

      LdsLvx *read_lvx = LdsLvx::GetInstance(1000 / publish_freq);
      lddc->RegisterLds(static_cast<Lds *>(read_lvx));
      lddc->CreateBagFile(rosbag_file_path);
      int ret = read_lvx->InitLdsLvx(cmdline_file_path.c_str());
      if (!ret) {
        ROS_INFO("Init lds lvx file success!");
      } else {
        ROS_ERROR("Init lds lvx file fail!");
      }
    } while (0);
  }

  ros::Time::init();
  while (ros::ok()) {
    lddc->DistributeLidarData();
  }

  // 正常退出（如果 while 循环结束而没收到信号）
  printf("\n=======================\n");
  printf("Shutting down via ROS shutdown...\n");
  SignalHandler(SIGUSR1);  // 使用信号处理函数来清理

  return 0;
}
