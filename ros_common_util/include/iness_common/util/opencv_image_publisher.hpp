/*
 * Copyright (C) Insightness AG, Switzerland - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Stefan Isler <stefan@insightness.com>
 * Mon Sep 18 2017
 */

#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "iness_common/time/definitions.hpp"
#include "iness_common/time/ros_conversions.hpp"

namespace iness
{
namespace util
{

/*! Convenience class to publish opencv Mat frames to ROS topics
 */
class OpenCvImagePublisher
{
public:
  /*! Constructor
   * @param _topic_name Name under which the image will be published on the namespace of the ros node handle
   * @param _nh The ros nodehandle that will be used to publish the images (defaults to private "~")
   * @param _encoding The image encoding (defaults to bgr)
   */
  OpenCvImagePublisher( std::string _topic_name, std::string _encoding = "bgr8", ros::NodeHandle _nh = ros::NodeHandle("~") );

  //! Name accessor
  std::string topicName() const;

  /*! Publishes one image to ROS
   * @param _image The image.
   * @param _time Time that will be used as timestamp. Defaults to the current ros time.
   */
  void publish( cv::Mat _image, iness::time::TimeUs _time = iness::time::usFromRos(ros::Time::now())) const;

  void publish( cv::Mat _image, ros::Time time) const;
  
  //! Sets a new image encoding
  void setEncoding( std::string _encoding );

  //! Sets a new node handle on which messages shall be published
  void setNodeHandle( const ros::NodeHandle& _nh );

  //! Sets a topic name under which to publish.
  void setTopicName( std::string _name );

private:
  ros::NodeHandle nh_; //!< Node handle on which topics are published.
  ros::Publisher image_pub_; //!< Image publisher.
  std::string topic_name_; //!< Name under which messages are published.
  std::string encoding_; //!< Encoding of the image.
};

}
}
