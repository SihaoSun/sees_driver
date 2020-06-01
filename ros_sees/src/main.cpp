#include <chrono>
#include <iostream>
#include <mutex>

#include <memory>
#include <ros/ros.h>

#include "iness_common/device/ros_sees/ros_sees_publisher.hpp"
#include "iness_common/device/ros_sees/ros_sees_publisher_mod.hpp"
#include "iness_common/device/sees/sees.hpp"
#include "ros_sees/ros_seesConfig.h"
#include <dynamic_reconfigure/server.h>

using iness::PolarityEvent;

std::shared_ptr<iness::device::Sees> driver_ptr;

void dynamicReconfigureCallback(ros_sees::ros_seesConfig &_config,
                                uint32_t _level) {
  iness::device::Sees::Config config;
  config.enable_aps = _config.aps_enabled;
  config.enable_dvs = _config.dvs_enabled;
  config.enable_imu = _config.imu_enabled;
  config.exposure_time_us = _config.exposure;
  config.frame_rate_hz = _config.frame_rate;
  config.accelerometer_scale = _config.accelerometer_scale;
  config.gyroscope_scale = _config.gyro_scale;

  driver_ptr->loadConfiguration(config);
}

int main(int argc, char **argv) {
  // init ros node
  ros::init(argc, argv, "sees_driver");

  // advertise topics
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  auto ns = ros::this_node::getNamespace();

  iness::device::DeviceRequirements device_req;
  nh_private.getParam("serial_number", device_req.serial_nr);

  // start driver
  std::shared_ptr<iness::device::Sees> driver =
      std::make_shared<iness::device::Sees>(device_req);

  std::cout << "Serial Number: " << driver->serialNumber() << std::endl;

  std::string config_file;
  nh_private.param("sees_config_file", config_file, config_file);

  iness::RosSeesPublisherMod ros_publisher("");
  ros_publisher.setDvsSize(driver->dvsWidth(), driver->dvsHeight());

  driver->registerCallback(
      std::bind(&iness::RosSeesPublisherMod::packetCallback, &ros_publisher,
                std::placeholders::_1));

  // Setup dynamic reconfigure
  driver_ptr = driver;
  dynamic_reconfigure::Server<ros_sees::ros_seesConfig> server(nh_private);
  dynamic_reconfigure::Server<ros_sees::ros_seesConfig>::CallbackType callback =
      boost::bind(&dynamicReconfigureCallback, _1, _2);
  server.setCallback(callback);

  if (config_file != "")
    driver->loadConfiguration(config_file);

  if (driver->start()) {
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Duration(120.0).sleep();
  }
  ros::shutdown();
  driver->stop();

  return 0;
}
