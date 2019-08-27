#ifndef KEY_FRAME_H
#define KEY_FRAME_H

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lidar_slam_3d
{

struct PosePQ{
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class KeyFrame
{
public:
    // 重载KeyFrame的new函数，使得class的内存空间对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<KeyFrame> Ptr;
    KeyFrame() {}
    KeyFrame(int id, const Eigen::Matrix4f& pose, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    {
        id_ = id;
        pose_ = pose;
        cloud_ = cloud;
    }
    ~KeyFrame() {}

    void setId(int id) { id_ = id; }
    void setPose(const Eigen::Matrix4f& pose) { pose_ = pose; }
    // 从变换矩阵获得位置和姿态
    void setPosePQ(const Eigen::Matrix4f& pose){
        pq_.p = pose.block<3,1>(0,3).template cast<double>(); // 取出t部分：起点是0行3列，取3行1列
        pq_.q = Eigen::Quaterniond(pose.block<3,3>(0,0).template cast<double>()); // 取出R部分：起点是0行0列，取3行3列
    }
    void setCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) { cloud_ = cloud; }

    int getId() { return id_; }
    Eigen::Matrix4f getPose() { return pose_; }
    // 返回PQ
    PosePQ getPosePQ(){return pq_;}
    pcl::PointCloud<pcl::PointXYZI>::Ptr getCloud() { return cloud_; }

private:
    int id_;
    Eigen::Matrix4f pose_;
    // 用于ceres优化的位姿
    PosePQ pq_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
};

} // namespace lidar_slam_3d

#endif // KEY_FRAME_H
