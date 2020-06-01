#ifndef INESS_COMMON_GEOMETRY_TYPES_H_
#define INESS_COMMON_GEOMETRY_TYPES_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iness_common/types/global.hpp>
#include "iness_common/time/definitions.hpp"

namespace iness
{

namespace geometry
{

typedef iness::Float FloatType;

/* Primitive type definitions */
typedef Eigen::Matrix<FloatType, 3, 4> ProjMat; // Camera Projection Matrix
typedef Eigen::Matrix<FloatType, 3, 3> CameraMat; // Camera Intrinsics Matrix
typedef Eigen::Matrix<FloatType, 3, 3> RotMat; // Rotation Matrix
typedef Eigen::Quaternion<FloatType> Quaternion;  // Quaternion
typedef Eigen::Matrix<FloatType, 3, 1> TransVec; // Translation Vector
typedef Eigen::Matrix<FloatType, 3, 1> LinearVel; // Linear Velocity Vector
typedef Eigen::Matrix<FloatType, 3, 1> AngularVel; // Angular Velocity Vector
typedef Eigen::Matrix<FloatType, 3, 1> Position; // Position vector
typedef Quaternion Attitude; // Attitude type

typedef Eigen::Matrix<int,2,1> Point2i;
typedef Eigen::Matrix<FloatType,2,1> Point2f;
typedef Eigen::Matrix<FloatType,2,1> Vector2f;

typedef Eigen::Matrix<FloatType, 3, 1> Point3f;
typedef Eigen::Matrix<FloatType, 3, 1> Vector3f;

/* Geometric Quantities */
typedef Eigen::Matrix<FloatType, 3, 1> Point3D; // 3D Point
typedef Eigen::Matrix<FloatType, 4, 1> Point3DHom; // Homogeneous 3D point
typedef std::pair<Point3DHom, Point3DHom> Line3DHom; // 3D line with homogeneous 3D end points
typedef Eigen::Matrix<FloatType, 3, 1> Point2DHom; // Homogeneous 2D point
typedef Eigen::Matrix<FloatType, 3, 1> Line2D; // 2D Line
typedef Eigen::Matrix<FloatType, 6, 1> PluckerLine; // 3D Pluecker Line with [m,l]
typedef Eigen::Matrix<FloatType, 4, 4> DualPluckerMat; // 3D Pluecker Line Dual Matrix
typedef Eigen::Matrix<FloatType, 4, 1> Plane3D; // 3D Plane


/*! Pose definition */
struct Pose
{
  Position pos; //! Position.
  Attitude ori; //! Orientation.

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct StampedPosition
{
  Position position;
  time::TimeUs time_us;
};

struct StampedAttitude
{
  Attitude attitude;
  time::TimeUs time_us;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct StampedPose
{
  Pose pose;
  time::TimeUs time_us;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/*! Probabilistic pose definition */
struct ProbPose
{
  Position pos; //! Position
  Attitude ori; //! Orientation
  Eigen::Matrix<FloatType,6,6> cov; //! Covariance matrix.

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
/*! Twist definition */
struct Twist
{
  LinearVel linear; //! Linear velocity
  AngularVel angular; //! Angular velocity

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct StampedOdometry
{
  Pose pose;
  Twist twist;
  time::TimeUs time_us;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}

}



#endif
