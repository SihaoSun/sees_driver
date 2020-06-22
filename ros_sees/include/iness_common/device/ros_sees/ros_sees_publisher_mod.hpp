/*
 * Copyright (C) Insightness AG, Switzerland - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Stefan Isler <stefan@insightness.com>
 * Tue Dec 05 2017
 */

#ifndef INESS_COMMON_SEES0_ROS_SEES0_ROS_SEES_PUBLISHER_MOD_HPP
#define INESS_COMMON_SEES0_ROS_SEES0_ROS_SEES_PUBLISHER_MOD_HPP

#include <mutex>

#include <Eigen/Core>
#include <chrono>

#include "iness_common/types/events/i_event_packet.hpp"
#include "iness_common/types/events/ser_events/event_packet.h"
#include "iness_common/types/global.hpp"
#include "iness_common/util/opencv_image_publisher.hpp"

#include "iness_common/device/sees/sees.hpp"

#include "ros/ros.h"
//#include <dvs_msgs/EventArray.h>
#include <ros_dvs_msgs/EventImuArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

namespace iness {

/*! Util class to ease publishing of ROS topics from sees data, offers the
 * needed event packet callbacks to interface with the driver.
 */
class RosSeesPublisherMod {
public:
  static constexpr double STANDARD_GRAVITY = 9.81;

public:
  //! Constructor
  RosSeesPublisherMod( std::string device,std::string _polarity_topic = "events",
                      std::string _frame_topic = "image_raw",
                      std::string _imu6_topic = "imu",
                      std::string _temperature_topic = "temperature",
                      ros::NodeHandle _nh = ros::NodeHandle("~"));

  //! Function to set the dvs frame size
  void setDvsSize(unsigned int _width, unsigned int _height);

  //! Callback function for IEvent packets
  void packetCallback(aedat::IEventPacket::Ptr _packet);

  //! Callback function for polarity event packets.
  void polarityEventPacketCallback(iness::PolarityEventPacket &_packet);

  //! Callback function for frame event packets.
  void frameEventPacketCallback(iness::FrameEventPacket &_packet);

  //! Callback function for imu6 event packets.
  void imu6EventPacketCallback(iness::Imu6EventPacket &_packet);

  //! Callback function for imu6 event packets.
  void specialEventPacketCallback(iness::SpecialEventPacket &_packet) const;

  //! Callback function for publishing IEventPackets as auxiliary data.
  void auxiliaryEventPacketCallback(aedat::IEventPacket::Ptr _packet) const;

  //! Can be used to set an imu coordinate transform (rotation)
  void setImuTransform(const Eigen::Matrix<iness::Float, 3, 3> &_rotation);

  //! Sets filter values
  void setImageFilter(Float _contrast = 1.0, Float _brightness = 0.0,
                      Float _gamma = 1.0,
                      bool _do_auto_min_max_adaption = false);

private:
  ros::NodeHandle nh_p_; //! Private node handle
  ros::Publisher polarity_publisher_;
  ros::Publisher imu6_publisher_;
  // ros::Publisher frame_publisher_;
  ros::Publisher temperature_publisher_;
  ros::Publisher special_events_publisher_;
  ros::Publisher
      auxiliary_publisher_; // Used to publish unknown AEDAT packet types.

  iness::util::OpenCvImagePublisher frame_publisher_;
  std::string imu6_topic_; //! Topic name with which imu6 events will be published under the node.

  Float contrast_{1.0};     //! Image contrast adjustment.
  Float brightness_{0.0};   //! Image brightness adjustment.
  Float gamma_factor_{1.0}; //! Image gamma factor adjustment.
  bool auto_min_max_adaption_{
      false}; //! If true all images are scaled such that they spawn the
              //! complete image range.

  bool do_imu_transformation_{false}; //! Whether an imu coordinate transform
                                      //! should be performed or not.
  Eigen::Matrix<iness::Float, 3, 3>
      imu_coordinate_rotation_; //! Imu coordinate transform to be applied.

  mutable ros_dvs_msgs::EventArray
      event_array_message_; //! Buffer for polarity events

  ros::Publisher event_imu_publisher_;
  // dvs_msgs::EventImuArray event_imu_array_;

  ros::Publisher event_publisher_;
  ros::Publisher imu_publisher_;

  std::vector<sensor_msgs::Imu> imu_array_;
  std::mutex imu_array_mutex_;
  std::mutex event_array_mutex_;

  int update_frequency_;
  ros::Timer update_timer_;
  void updateLoop(const ros::TimerEvent &time);
};

} // namespace iness

#endif // INESS_COMMON_SEES0_ROS_SEES0_ROS_SEES_PUBLISHER_HPP
