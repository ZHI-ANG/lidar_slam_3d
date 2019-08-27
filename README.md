# lidar_slam_3d

forked from <https://github.com/ningwang1028/lidar_slam_3d>

代码结构清晰明了，适合作为框架进行优化。初步计划：

1. 优化回环检测
2. 添加imu，实现预积分
3. 修改为ceres版本，进一步变为不需要ceres的版本

官方给出的效果图如下：

<img src="image/map.png" height="512pix" /> 

### 更新日志

#### 2019.08.27 将Pose Graph优化器由g2o修改为ceres求解器

1. 最近在学习ceres库，因此正好用这个代码练练手。这部分的代码基本照搬ceres官方给出的slam_3d

2. ceres给出的代码范例中，后端优化的Pose Graph分别使用Eigen::Vector3d表示位置，Eigen::Quaternion表示旋转；而原程序代码中，Pose Graph使用Eigen::Isometry3d(4*4 Matrix)表示位姿。为了以后参考VINS添加IMU预积分，这里将前端的数据接口修改为Eigen::Vector3d和Eigen::Quaternion