/*
 * Copyright (C) Insightness AG, Switzerland - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Stefan Isler <stefan@insightness.com>
 * Tue Dec 05 2017
 */

#include "iness_common/device/ros_sees/ros_sees_publisher_mod.hpp"
#include "iness_common/geometry/types.hpp"

#include <ros_dvs_msgs/AedatPacket.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

#include <algorithm>

namespace iness {

RosSeesPublisherMod::RosSeesPublisherMod( std::string device,std::string _polarity_topic,
                                         std::string _frame_topic,
                                         std::string _imu6_topic,
                                         std::string _temperature_topic,
                                         ros::NodeHandle _nh)
    : frame_publisher_(device + _frame_topic,"mono8",_nh),
 imu6_topic_(_imu6_topic),
      nh_p_(_nh) {
   nh_p_.param<int>("update_frequency", update_frequency_, 200);

  std::string event_imu_topic;
  nh_p_.param<std::string>("event_imu_topic", event_imu_topic, "event_imu");

  event_imu_publisher_ =
      nh_p_.advertise<ros_dvs_msgs::EventImuArray>(device + event_imu_topic, 1);
  // frame_publisher_ = nh_p_.advertise<sensor_msgs::Image>("image_raw", 1);

  event_publisher_ = nh_p_.advertise<ros_dvs_msgs::EventArray>("/sees/events",10);
  imu_publisher_ = nh_p_.advertise<sensor_msgs::Imu>("/sees/imu",10);

  event_array_message_.width = 346;
  event_array_message_.height = 260;

  imu_array_.reserve(10000);
  imu_array_mutex_.unlock();
  event_array_message_.events.reserve(1000000);
  event_array_mutex_.unlock();

  update_timer_ = nh_p_.createTimer(ros::Rate(update_frequency_),
                                    &RosSeesPublisherMod::updateLoop, this);
}

void RosSeesPublisherMod::setDvsSize(unsigned int _width,
                                     unsigned int _height) {
  event_array_message_.width = _width;
  event_array_message_.height = _height;
}

void RosSeesPublisherMod::packetCallback(aedat::IEventPacket::Ptr _packet) {
  switch (_packet->header()->event_type) {
  case serialization::AedatType::POLARITY_EVENT:
    polarityEventPacketCallback(
        *_packet->interpretAs<iness::PolarityEventPacket>());
    break;
  case serialization::AedatType::FRAME_EVENT:
    frameEventPacketCallback(*_packet->interpretAs<iness::FrameEventPacket>());
    break;
  case serialization::AedatType::IMU6_EVENT:
    imu6EventPacketCallback(*_packet->interpretAs<iness::Imu6EventPacket>());
    break;
  case serialization::AedatType::SPECIAL_EVENT:
    // specialEventPacketCallback(
    //    *_packet->interpretAs<iness::SpecialEventPacket>());
    break;
  default:
    // auxiliaryEventPacketCallback(_packet);
    break;
  };
}

void RosSeesPublisherMod::polarityEventPacketCallback(
    iness::PolarityEventPacket &_packet) {
  event_array_mutex_.lock();
  for (auto &event : _packet) {
    ros_dvs_msgs::Event e;
    e.x = event.getX();
    e.y = event.getY();
    e.ts = iness::time::toRos(event.getTimestampUs(_packet.tsOverflowCount()));
    e.polarity = (bool)event.getPolarity();
    event_array_message_.events.push_back(e);
  }
  event_array_mutex_.unlock();
}

void RosSeesPublisherMod::frameEventPacketCallback(
  iness::FrameEventPacket &_packet){
  for (auto &event : _packet) {
    cv::Mat img = event.getImage(); // is 16bit
    img.convertTo(img, CV_32FC1,
                  0.000015319); // from 0...2^17-1 to 0...1 range

    if (auto_min_max_adaption_)
      cv::normalize(img, img, 0, 1.0, cv::NORM_MINMAX);
    img = contrast_ * img + brightness_;
    cv::pow(img, gamma_factor_, img);
    img.convertTo(img, CV_8UC1, 255.0); // convert to 8bit

    // frame_publisher_.publish(img, event.getTimestampUs(_packet.tsOverflowCount()) );
    frame_publisher_.publish(img, ros::Time::now());
  }
}

void RosSeesPublisherMod::imu6EventPacketCallback(
    iness::Imu6EventPacket &_packet) {
  for (auto &event : _packet) {
    sensor_msgs::Imu imu_msg;

    iness::geometry::Vector3f acceleration(event.getAccelerationX(),
                                           event.getAccelerationY(),
                                           event.getAccelerationZ());
    acceleration *= STANDARD_GRAVITY; // converts to m/s^2

    iness::geometry::Vector3f angular_velocity(
        event.getGyroX(), event.getGyroY(), event.getGyroZ());
    angular_velocity *= (M_PI / 180.0); // convert from deg/s to rad/s

    if (do_imu_transformation_) {  //???
      acceleration = imu_coordinate_rotation_ * acceleration;
      angular_velocity = imu_coordinate_rotation_ * angular_velocity;
    }

    imu_msg.linear_acceleration.x = acceleration(0);
    imu_msg.linear_acceleration.y = acceleration(1);
    imu_msg.linear_acceleration.z = acceleration(2);
    imu_msg.angular_velocity.x = angular_velocity(0);
    imu_msg.angular_velocity.y = angular_velocity(1);
    imu_msg.angular_velocity.z = angular_velocity(2);

    // no orientation estimate:
    // http://docs.ros.org/api/sensor_msgs/html/imu_msg/Imu.html
    imu_msg.orientation_covariance[0] = -1.0;

    // time
    imu_msg.header.stamp =
        iness::time::toRos(event.getTimestampUs(_packet.tsOverflowCount()));

    imu_msg.header.frame_id = imu6_topic_;

    if (imu_publisher_.getNumSubscribers()>0)
      imu_publisher_.publish(imu_msg);
  }
}

void RosSeesPublisherMod::auxiliaryEventPacketCallback(
    aedat::IEventPacket::Ptr _packet) const {
  if (auxiliary_publisher_.getNumSubscribers() != 0) {
    ros_dvs_msgs::AedatPacket aedat_msg;

    aedat_msg.header.stamp =
        iness::time::toRos(_packet->getFirstEventTimestampUs());
    aedat_msg.header.frame_id = "aedat_auxiliary";

    aedat_msg.aedat_type =
        (decltype(aedat_msg.aedat_type))_packet->header()->event_type;
    aedat_msg.event_source = _packet->header()->event_source;

    char *aedat_data_begin = _packet->data();
    size_t packet_size =
        sizeof(iness::EventPacketHeader) +
        _packet->header()->event_nr * _packet->header()->event_size;
    char *aedat_data_end = aedat_data_begin + packet_size;

    aedat_msg.data.resize(packet_size, 0);
    std::copy(aedat_data_begin, aedat_data_end, aedat_msg.data.begin());

    auxiliary_publisher_.publish(aedat_msg);
  }
}

void RosSeesPublisherMod::setImuTransform(
    const Eigen::Matrix<iness::Float, 3, 3> &_rotation) {
  do_imu_transformation_ = true;
  imu_coordinate_rotation_ = _rotation;
}

void RosSeesPublisherMod::setImageFilter(Float _contrast, Float _brightness,
                                         Float _gamma,
                                         bool _auto_min_max_adaption) {
  contrast_ = _contrast;
  brightness_ = _brightness;
  gamma_factor_ = _gamma;
  auto_min_max_adaption_ = _auto_min_max_adaption;
}

void RosSeesPublisherMod::updateLoop(const ros::TimerEvent &time) {
  event_array_mutex_.lock();

  if (event_publisher_.getNumSubscribers()>0)
    event_publisher_.publish(event_array_message_);

  event_array_message_.events.clear();
  
  event_array_mutex_.unlock();
}
} // namespace iness
