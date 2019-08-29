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

typedef std::map<int, PosePQ, std::less<int>, Eigen::aligned_allocator<std::pair<const int, PosePQ> > >
    MapOfPoses;


} //namespace lidar_slam_3d
#endif
