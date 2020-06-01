#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

#include <memory>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "iness_common/device/ros_sees/ros_sees_publisher.hpp"
#include "iness_common/device/ros_sees/ros_sees_publisher_mod.hpp"
#include "iness_common/device/sees/sees.hpp"
#include "ros_sees/ros_seesConfig.h"
#include <dynamic_reconfigure/server.h>

using iness::PolarityEvent;

void driverThread(const iness::device::DeviceRequirements device_req, const std::string device, const std::string config) {
  // start driver
  std::shared_ptr<iness::device::Sees> driver =
      std::make_shared<iness::device::Sees>(device_req);

  std::cout << "Serial Number: " << driver->serialNumber() << std::endl;

  iness::RosSeesPublisherMod ros_publisher(device);
  ros_publisher.setDvsSize(driver->dvsWidth(), driver->dvsHeight());

  driver->registerCallback(
      std::bind(&iness::RosSeesPublisherMod::packetCallback, &ros_publisher,
  std::placeholders::_1));

  if (config != "")
    driver->loadConfiguration(config);

  if (driver->start()) {
    while (ros::ok())
    {
      ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.005));
    }
  }
  driver->stop();     
}

int main(int argc, char **argv) {
  // init ros node
  ros::init(argc, argv, "sees_driver");

  // advertise topics
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  iness::device::DeviceRequirements device_req_1;
  nh_private.getParam("serial_number_1", device_req_1.serial_nr);
  iness::device::DeviceRequirements device_req_2;
  nh_private.getParam("serial_number_2", device_req_2.serial_nr);

  std::string config_file;
  nh_private.param("sees_config_file", config_file, config_file);

  std::cout << "Starting Threads" << std::endl;

  std::thread device_1_thread(driverThread, device_req_1, "cam_1/", config_file);
  std::thread device_2_thread(driverThread, device_req_2, "cam_2/", config_file);

  std::cout << "Started" << std::endl;
  int i = 0;
  while(ros::ok() && i < 120) {
    ros::Duration(1.0).sleep();
    ++i;
    std::cout << i << std::endl;
  }
  std::cout << "Shutting Down" << std::endl;
  ros::shutdown();
  std::cout << "Finished" << std::endl;
//  ros::waitForShutdown();

  return 0;
}
