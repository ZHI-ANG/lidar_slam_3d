#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <map>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lidar_slam_3d{
struct PosePQ{
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct LoopConstraint{
    int first_;
    int second_;
    Eigen::Matrix4f loop_pose_; // 当前关键帧在回环点云中的位姿
};

} //namespace lidar_slam_3d
#endif
