#pragma once

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

#include <memory>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "iness_common/device/ros_sees/ros_sees_publisher_mod.hpp"
#include "iness_common/device/sees/sees.hpp"
#include "ros_sees/ros_seesConfig.h"
#include <dynamic_reconfigure/server.h>

#include <rpg_common/main.h>

namespace iness {
class StereoClass
{
public:
  StereoClass ();
  ~StereoClass () {}

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  iness::device::DeviceRequirements device_req_1_;
  iness::device::DeviceRequirements device_req_2_;

  std::string config_file_;

  std::thread device_1_thread_;
  std::thread device_2_thread_;

  ros::MultiThreadedSpinner spinner_;

  void loadParameters();
  void threadFunc1();
  void threadFunc2();
};
}
