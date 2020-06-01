#ifndef INESS_COMMON_TIME_ROS_CONVERSIONS_H_
#define INESS_COMMON_TIME_ROS_CONVERSIONS_H_

#include "iness_common/time/definitions.hpp"
#include <ros/ros.h>

namespace iness
{
namespace time
{

/*! Converts the insightness time format into ROS time format. (Bare conversion, no offsets or other synchronization is carried out)
 *
 */
inline ros::Time toRos( TimeUs _time )
{
	ros::Time time;
	time.fromNSec(_time*1000);
	return time;
}

/*! Converts the ROS time format into the Insightness time format. (Bare conversion, noffsets or other synchronizatino is carried out)
 *
 */
inline TimeUs usFromRos( ros::Time _time )
{
	return _time.toNSec()/1000;

}

}
}


#endif
