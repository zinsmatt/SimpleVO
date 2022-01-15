#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>

typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector2d Vec2;
typedef Eigen::Matrix3d Mat33;
typedef Eigen::Matrix4d Mat44;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;

typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

#include <glog/logging.h>

#include <opencv2/opencv.hpp>

#include <iostream>

#endif // COMMON_H