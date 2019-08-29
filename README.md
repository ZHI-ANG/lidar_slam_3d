# lidar_slam_3d

forked from <https://github.com/ningwang1028/lidar_slam_3d>

代码结构清晰明了，提供了一个优秀的基本框架，每一部分的接口相对独立，方便进行进一步的优化。

原工程给出的效果图如下：

<img src="image/map.png" height="512pix" />

#### 初步计划：

3. 求解器修改为ceres版本
2. 添加IMU预积分

#### 可能的计划：

1. 手写求解器
2. 添加IMU和激光的时间戳配准 



### 更新日志

#### 2019.08.27 将位姿表示由Transformation替换为Pose+Quaternion

1. 最近在学习ceres库，因此正好用这个代码练练手。这部分的代码基本照搬ceres官方给出的slam_3d

2. ceres给出的代码范例中，后端优化的Pose Graph分别使用Eigen::Vector3d表示位置，Eigen::Quaternion表示旋转；而原程序代码中，Pose Graph使用Eigen::Isometry3d(4*4 Matrix)表示位姿。为了以后参考VINS添加IMU预积分，这里将前端的数据接口修改为Eigen::Vector3d和Eigen::Quaternion

#### 2019.08.29 将图优化由g2o求解器替换为ceres求解器

1. 将g2o替换成ceres，最重要的是**建立map的映射，方便对位姿顶点的管理**。
2. 将addEdge()和addVertex()函数改为addResidualBlock()函数，管理ceres使用的位姿顶点，由于每个顶点不止一个约束(观测)，因此需要添加一个新的costfunction
3. 在修改代码的过程中发现，原程序的位姿图优化仅仅存在于回环过程中，相邻帧之间不做优化，代码中相邻帧的观测和预测完全相等，因此优化没有意义。但是当检测到回环时，会将迄今的所有位姿点添加进位姿图优化中，进行优化。

