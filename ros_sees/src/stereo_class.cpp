#include "iness_common/device/ros_sees/stereo_class.hpp"

namespace iness {
StereoClass::StereoClass() : nh_(ros::NodeHandle()), nh_private_(ros::NodeHandle("~")), spinner_(ros::MultiThreadedSpinner(2)) {
  loadParameters();
  
  device_1_thread_ = std::thread(&StereoClass::threadFunc1, this);
  device_2_thread_ = std::thread(&StereoClass::threadFunc2, this);
}

void StereoClass::loadParameters() {
  nh_private_.getParam("serial_number_1", device_req_1_.serial_nr);
  nh_private_.getParam("serial_number_2", device_req_2_.serial_nr);
  nh_private_.param("sees_config_file", config_file_, config_file_);

}

void StereoClass::threadFunc1() {
  std::shared_ptr<iness::device::Sees> driver =
      std::make_shared<iness::device::Sees>(device_req_1_);

  std::cout << "Serial Number: " << driver->serialNumber() << std::endl;

  iness::RosSeesPublisherMod ros_publisher("cam_1/");
  ros_publisher.setDvsSize(driver->dvsWidth(), driver->dvsHeight());

  driver->registerCallback(
      std::bind(&iness::RosSeesPublisherMod::packetCallback, &ros_publisher,
  std::placeholders::_1));

  if (config_file_ != "")
    driver->loadConfiguration(config_file_);

  if (driver->start()) {
    spinner_.spin();
  }
  driver->stop();     
}

void StereoClass::threadFunc2() {
  std::shared_ptr<iness::device::Sees> driver =
      std::make_shared<iness::device::Sees>(device_req_2_);

  std::cout << "Serial Number: " << driver->serialNumber() << std::endl;

  iness::RosSeesPublisherMod ros_publisher("cam_2/");
  ros_publisher.setDvsSize(driver->dvsWidth(), driver->dvsHeight());

  driver->registerCallback(
      std::bind(&iness::RosSeesPublisherMod::packetCallback, &ros_publisher,
  std::placeholders::_1));

  if (config_file_ != "")
    driver->loadConfiguration(config_file_);

  if (driver->start()) {
    spinner_.spin();
  }
  driver->stop();     
}

}

RPG_COMMON_MAIN
{
  ros::init(argc, argv, "sees_driver");
  iness::StereoClass stereo;
  ros::waitForShutdown();
  return 0;
}
