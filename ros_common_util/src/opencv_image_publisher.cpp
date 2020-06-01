/*
 * Copyright (C) Insightness AG, Switzerland - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Stefan Isler <stefan@insightness.com>
 * Mon Sep 18 2017
 */

#include "iness_common/util/opencv_image_publisher.hpp"

namespace iness
{
namespace util
{

OpenCvImagePublisher::OpenCvImagePublisher( std::string _topic_name, std::string _encoding, ros::NodeHandle _nh )
: nh_(_nh)
, topic_name_(_topic_name)
, encoding_(_encoding)
{
  image_pub_ = nh_.advertise<sensor_msgs::Image>(topic_name_, 100);
}

std::string OpenCvImagePublisher::topicName() const
{
  return topic_name_;
}

void OpenCvImagePublisher::publish( cv::Mat _image, ros::Time _time ) const
{
  if( image_pub_.getNumSubscribers()!=0 )
  {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encoding_, _image).toImageMsg();
    msg->header.stamp = _time;
    image_pub_.publish(msg);
  }
}

void OpenCvImagePublisher::publish( cv::Mat _image, iness::time::TimeUs _time ) const
{
  if( image_pub_.getNumSubscribers()!=0 )
  {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encoding_, _image).toImageMsg();
    msg->header.stamp = iness::time::toRos(_time);
    image_pub_.publish(msg);
  }
}

void OpenCvImagePublisher::setEncoding( std::string _encoding )
{
  encoding_ = _encoding;
}

void OpenCvImagePublisher::setNodeHandle( const ros::NodeHandle& _nh )
{
  nh_ = _nh;
  image_pub_ = nh_.advertise<sensor_msgs::Image>(topic_name_, 100);
}

void OpenCvImagePublisher::setTopicName( std::string _name )
{
  topic_name_ = _name;
}


}
}
