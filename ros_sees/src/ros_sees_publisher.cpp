/*
 * Copyright (C) Insightness AG, Switzerland - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Stefan Isler <stefan@insightness.com>
 * Tue Dec 05 2017
 */


#include "iness_common/device/ros_sees/ros_sees_publisher.hpp"
#include "iness_common/geometry/types.hpp"

#include <ros_dvs_msgs/AedatPacket.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

#include <algorithm>

namespace iness
{

RosSeesPublisher::RosSeesPublisher(std::string device,
    std::string _polarity_topic, 
                                  std::string _frame_topic, 
                                  std::string _imu6_topic, 
				  std::string _temperature_topic,
                                  ros::NodeHandle _nh )
: polarity_topic_(_polarity_topic)
, frame_topic_(_frame_topic)
, imu6_topic_(_imu6_topic)
, temperature_topic_(_temperature_topic)
, nh_p_(_nh)
, frame_publisher_(_frame_topic,"mono8",_nh)
  , time_initialized_(false)
{
  special_events_topic_ = "special_events";
  auxiliary_events_topic_ = "aedat_auxiliary";
  
  polarity_publisher_ = nh_p_.advertise<ros_dvs_msgs::EventArray>(polarity_topic_, 1000);
  imu6_publisher_ = nh_p_.advertise<sensor_msgs::Imu>(imu6_topic_, 1000);
  temperature_publisher_ = nh_p_.advertise<sensor_msgs::Temperature>(temperature_topic_, 1000);
  special_events_publisher_ = nh_p_.advertise<ros_dvs_msgs::AedatPacket>(special_events_topic_, 1000);
  auxiliary_publisher_ = nh_p_.advertise<ros_dvs_msgs::AedatPacket>(auxiliary_events_topic_, 1000);

  event_array_message_.width = 346;
  event_array_message_.height = 260;

}

void RosSeesPublisher::setDvsSize( unsigned int _width, unsigned int _height )
{
  event_array_message_.width = _width;
  event_array_message_.height = _height;
}

void RosSeesPublisher::packetCallback( aedat::IEventPacket::Ptr _packet ) const
{
  switch( _packet->header()->event_type )
  {
    case serialization::AedatType::POLARITY_EVENT:
      polarityEventPacketCallback( *_packet->interpretAs<iness::PolarityEventPacket>() );
      break;
    case serialization::AedatType::FRAME_EVENT:
      frameEventPacketCallback( *_packet->interpretAs<iness::FrameEventPacket>() );
      break;
    case serialization::AedatType::IMU6_EVENT:
      imu6EventPacketCallback( *_packet->interpretAs<iness::Imu6EventPacket>() );
      break;
    case serialization::AedatType::SPECIAL_EVENT:
      specialEventPacketCallback( *_packet->interpretAs<iness::SpecialEventPacket>() );
      break;
    default:
      auxiliaryEventPacketCallback( _packet );
  };
}

void RosSeesPublisher::polarityEventPacketCallback( iness::PolarityEventPacket& _packet ) const
{
  if( polarity_publisher_.getNumSubscribers() != 0 )
  {
    for(auto& event : _packet)
    {
        ros_dvs_msgs::Event e;
        e.x = event.getX();
        e.y = event.getY();
        e.ts = iness::time::toRos( event.getTimestampUs(_packet.tsOverflowCount() ));
        e.polarity = (bool)event.getPolarity();
        event_array_message_.events.push_back(e);
    }
    polarity_publisher_.publish(event_array_message_);
  }
  event_array_message_.events.clear();
}

void RosSeesPublisher::frameEventPacketCallback( iness::FrameEventPacket& _packet ) const
{
  for( auto& event : _packet )
  {
    cv::Mat img = event.getImage(); // is 16bit
    img.convertTo(img,CV_32FC1,0.000015319); // from 0...2^17-1 to 0...1 range

    if(auto_min_max_adaption_)
      cv::normalize(img,img,0,1.0,cv::NORM_MINMAX);
    img = contrast_*img + brightness_;
    cv::pow(img,gamma_factor_,img);
    img.convertTo(img,CV_8UC1,255.0); // convert to 8bit

    if (!time_initialized_) {
      start_time_ = ros::Time::now();
      start_time_sec_ = start_time_.toSec();
      time_initialized_ = true;
    }

    frame_publisher_.publish(img, event.getTimestampUs(_packet.tsOverflowCount())  + start_time_sec_* 1.0e6);
  }
}

void RosSeesPublisher::imu6EventPacketCallback( iness::Imu6EventPacket& _packet ) const
{
  if( imu6_publisher_.getNumSubscribers() != 0 )
  {
    for (auto& event : _packet)
    {
        sensor_msgs::Imu imu_msg;
	      sensor_msgs::Temperature temperature_msg;

        iness::geometry::Vector3f acceleration(event.getAccelerationX(), event.getAccelerationY(), event.getAccelerationZ());
        acceleration *= STANDARD_GRAVITY; // converts to m/s^2

        iness::geometry::Vector3f angular_velocity( event.getGyroX(), event.getGyroY(), event.getGyroZ() );
        angular_velocity *= (M_PI/180.0); // convert from deg/s to rad/s

        if(do_imu_transformation_)
        {
          acceleration = imu_coordinate_rotation_*acceleration;
          angular_velocity = imu_coordinate_rotation_*angular_velocity;
        }

        imu_msg.linear_acceleration.x = acceleration(0);
        imu_msg.linear_acceleration.y = acceleration(1);
        imu_msg.linear_acceleration.z = acceleration(2);
        imu_msg.angular_velocity.x = angular_velocity(0);
        imu_msg.angular_velocity.y = angular_velocity(1);
        imu_msg.angular_velocity.z = angular_velocity(2);

        // no orientation estimate: http://docs.ros.org/api/sensor_msgs/html/imu_msg/Imu.html
        imu_msg.orientation_covariance[0] = -1.0;

        // time
        imu_msg.header.stamp = iness::time::toRos(event.getTimestampUs(_packet.tsOverflowCount()));

        // frame
        imu_msg.header.frame_id = imu6_topic_;
	
	
        // TEMPERATURE MESSAGE **************************************************
        temperature_msg.header.stamp = imu_msg.header.stamp;
        temperature_msg.header.frame_id = imu6_topic_;
        temperature_msg.temperature = event.getTemperature();
        temperature_msg.variance = 0.0f; // TODO this is unknown

        imu6_publisher_.publish(imu_msg);
        temperature_publisher_.publish(temperature_msg);
    }
  }
}

void RosSeesPublisher::specialEventPacketCallback( iness::SpecialEventPacket& _packet ) const
{
  if( special_events_publisher_.getNumSubscribers() != 0 )
  {
    ros_dvs_msgs::AedatPacket aedat_msg;
    
    aedat_msg.header.stamp = iness::time::toRos(_packet.first().getTimestampUs(_packet.tsOverflowCount()));
    aedat_msg.header.frame_id = special_events_topic_;
    
    aedat_msg.aedat_type = (decltype(aedat_msg.aedat_type))_packet.type();
    aedat_msg.event_source = _packet.header().event_source;
    
    char* aedat_data_begin = reinterpret_cast<char*>(&_packet);
    size_t packet_size = sizeof(iness::EventPacketHeader) + _packet.header().event_nr*_packet.header().event_size;
    char* aedat_data_end = aedat_data_begin + packet_size;
    
    aedat_msg.data.resize(packet_size,0);
    std::copy( aedat_data_begin, aedat_data_end, aedat_msg.data.begin() );
    
    special_events_publisher_.publish(aedat_msg);
  }
}

void RosSeesPublisher::auxiliaryEventPacketCallback( aedat::IEventPacket::Ptr _packet) const
{
  if( auxiliary_publisher_.getNumSubscribers() != 0 )
  {
    ros_dvs_msgs::AedatPacket aedat_msg;
    
    aedat_msg.header.stamp = iness::time::toRos( _packet->getFirstEventTimestampUs() );
    aedat_msg.header.frame_id = "aedat_auxiliary";
    
    aedat_msg.aedat_type = (decltype(aedat_msg.aedat_type))_packet->header()->event_type;
    aedat_msg.event_source = _packet->header()->event_source;
    
    char* aedat_data_begin = _packet->data();
    size_t packet_size = sizeof(iness::EventPacketHeader) + _packet->header()->event_nr*_packet->header()->event_size;
    char* aedat_data_end = aedat_data_begin + packet_size;
    
    aedat_msg.data.resize(packet_size,0);
    std::copy( aedat_data_begin, aedat_data_end, aedat_msg.data.begin() );
    
    auxiliary_publisher_.publish(aedat_msg);
  }
}  

void RosSeesPublisher::setImuTransform( const Eigen::Matrix<iness::Float,3,3>& _rotation )
{
  do_imu_transformation_ = true;
  imu_coordinate_rotation_ = _rotation;
}

void RosSeesPublisher::setImageFilter( Float _contrast, Float _brightness, Float _gamma, bool _auto_min_max_adaption )
{
  contrast_ = _contrast;
  brightness_ = _brightness;
  gamma_factor_ = _gamma;
  auto_min_max_adaption_ = _auto_min_max_adaption;
}


}
