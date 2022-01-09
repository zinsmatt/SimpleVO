#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>

typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector2d Vec2;
typedef Eigen::Matrix3d Mat33;
typedef Eigen::Matrix4d Mat44;
typedef Eigen::Matrix<double, Eigen::Dynamic, 4> MatX4;

typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;

#include <sophus/se3.hpp>

typedef Sophus::SE3d SE3;

#include <glog/logging.h>



#endif // COMMON_H