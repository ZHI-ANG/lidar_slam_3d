#include "map_builder.h"
#include <ceres/ceres.h>
#include "pose_graph_error.h"

using ceres::AutoDiffCostFunction;
using ceres::Solver;
using ceres::Solve;
using ceres::CostFunction;

namespace lidar_slam_3d
{

// 默认构造函数，构造时认为是第一个点云，编号为0
MapBuilder::MapBuilder() :
    first_point_cloud_(true), sequence_num_(0),
    pose_(Eigen::Matrix4f::Identity()), last_update_pose_(Eigen::Matrix4f::Identity()),
    submap_size_(30), voxel_grid_leaf_size_(2.0), map_update_distance_(1.0), enable_optimize_(true),
    loop_search_distance_(20.0), loop_min_chain_size_(5), loop_min_fitness_score_(1.5),
    loop_keyframe_skip_(20), loop_constraint_count_(0), optimize_every_n_constraint_(10)
{
    // ndt算法参数
    ndt_.setTransformationEpsilon(0.01);
    ndt_.setStepSize(0.1);
    ndt_.setResolution(1.0);
    ndt_.setMaximumIterations(30);
}

// 采用voxelGridFilter对点云进行降采样
void MapBuilder::downSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr& sampled_cloud)
{
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_grid_leaf_size_, voxel_grid_leaf_size_, voxel_grid_leaf_size_);
    voxel_grid_filter.setInputCloud(input_cloud);
    voxel_grid_filter.filter(*sampled_cloud);
}

// 查找回环检测中最接近当前帧位置的历史帧
// 这里使用两帧之间的位置的二范数作为距离
KeyFrame::Ptr MapBuilder::getClosestKeyFrame(const KeyFrame::Ptr& key_frame,
                                             const std::vector<KeyFrame::Ptr>& candidates)
{
    Eigen::Vector3d pt1 = key_frame->getPose().block<3, 1>(0, 3).template cast<double>();
    float min_distance = std::numeric_limits<float>::max();
    int id;

    for(const KeyFrame::Ptr& frame : candidates) {
        Eigen::Vector3d pt2 = frame->getPose().block<3, 1>(0, 3).template cast<double>();
        float distance = (pt1 - pt2).norm();
        if(distance < min_distance) {
            min_distance = distance;
            id = frame->getId();
        }
    }

    return key_frames_[id];
}

// 回环检测
void MapBuilder::detectLoopClosure(const KeyFrame::Ptr& key_frame)
{
    std::vector<KeyFrame::Ptr> cloud_chain; // 包含关键帧的点云序列
    std::vector<std::vector<KeyFrame::Ptr>> cloud_chains; // 二维的点云序列

    // 取出位姿的位置信息
    int n = key_frames_.size();
    Eigen::Vector3d pt1 = key_frame->getPose().block<3, 1>(0, 3).template cast<double>();

    // 在历史关键帧中遍历，求出当前帧和历史帧位置的距离
    for(int i = 0; i < n; ++i) {
        Eigen::Vector3d pt2 = key_frames_[i]->getPose().block<3, 1>(0, 3).template cast<double>();
        float distance = (pt1 - pt2).norm();

        // 若历史关键帧kf[i]和当前关键帧的位置距离小于阈值，即可能存在回环
        // 同时该历史关键帧与当前关键帧的序列号之差超过了设定的最少回环帧数
        // 即kf.id - kfs[i].id > loop_kf_skip，将该历史帧添加进回环链
        // 一旦出现不满足上述条件的历史关键帧，将回环链清空，重新添加
        if(distance < loop_search_distance_) {
            if(key_frames_[i]->getId() < key_frame->getId() - loop_keyframe_skip_) {
                cloud_chain.push_back(key_frames_[i]);
            }
            else {
                cloud_chain.clear();
            }
        }
        // 若历史关键帧kf[i]和当前关键帧距离大于回环搜索距离，检查回环链中kf的数量是否满足局部地图建图的帧数要求
        // 1. 满足，则将该系列帧添加进局部地图链中
        // 2. 否则，不添加，清空
        else {
            if(cloud_chain.size() > loop_min_chain_size_) {
                cloud_chains.push_back(cloud_chain);
                std::cout << "\033[36m" << "Find loop candidates. Keyframe chain size "
                          << cloud_chain.size() << std::endl;
                cloud_chain.clear();
            }
            else {
                cloud_chain.clear();
            }
        }
        // 一开始的历史kf和当前kf的距离都会较远，先进入else的else，被清空
        // 之后出现历史kf和当前kf的距离满足loop_dis，进入if条件，同时满足最小回环帧差，cloud_chain开始增加
        // 然后出现历史kf和当前kf的距离不满足loop_dis，进入else的if条件，添加进cloud_chain
    }

    if(cloud_chains.empty()) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    downSample(key_frame->getCloud(), sampled_cloud);

    // 从多个回环链中遍历每一个回环链
    for(const std::vector<KeyFrame::Ptr>& chain : cloud_chains) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        // 构建局部地图，向target_cloud中添加每个关键帧的点云
        for(const KeyFrame::Ptr& frame : chain) {
            pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
            pcl::transformPointCloud(*(frame->getCloud()), transformed_cloud, frame->getPose());
            *target_cloud += transformed_cloud;
        }

        // 采用ndt算法，对当前关键帧的点云和局部地图进行匹配
        pcl::PointCloud<pcl::PointXYZI> output_cloud;
        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
        ndt.setTransformationEpsilon(1e-3);
        ndt.setStepSize(0.1);
        ndt.setResolution(1.0);
        ndt.setMaximumIterations(30);
        ndt.setInputSource(sampled_cloud); 
        ndt.setInputTarget(target_cloud);
        ndt.align(output_cloud, key_frame->getPose());

        Eigen::Matrix4f loop_pose = ndt.getFinalTransformation();

        bool converged = ndt.hasConverged();
        double fitness_score = ndt.getFitnessScore();
        int final_num_iteration = ndt.getFinalNumIteration();

        std::cout << "Loop registration fitness_score " << fitness_score << std::endl;

        // 回环过程中配准成功
        if(converged && fitness_score < loop_min_fitness_score_) {
            // 在回环中找到最接近的关键帧
            KeyFrame::Ptr closest_keyframe = getClosestKeyFrame(key_frame, chain);
            // 将回环检测得到的两帧的约束添加进回环队列中
            LoopConstraint lc; lc.first_ = key_frame->getId(); lc.second_ = closest_keyframe->getId();
            lc.loop_pose_ = loop_pose;
            vLoopConstraint.push_back(lc);

            loop_constraint_count_++;
            optimize_time_ = std::chrono::steady_clock::now();
            std::cout << "Add loop constraint." << std::endl;
        }
    }
}

// 判断回环检测是否需要优化，满足以下两个条件之一
// 1. 当回环中的约束计数大于阈值
// 2. 超过一定时间后
bool MapBuilder::needOptimize()
{
    if(loop_constraint_count_ > optimize_every_n_constraint_) {
        return true;
    }

    if(loop_constraint_count_ > 0) {
        auto delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(
                       std::chrono::steady_clock::now() - optimize_time_);
        if(delta_t.count() > 10.0) {
            return true;
        }
    }

    return false;
}

// 添加点云，并通过ndt计算出初始位姿
// 接收key_frame传递的一个过滤过的点云
// 每次添加点云，都会向优化器添加该点云的位姿
// 然后检测是否是回环位置
void MapBuilder::addPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud)
{
    auto t1 = std::chrono::steady_clock::now();
    sequence_num_++;

    // 若是第一个点云，则仅仅添加进key_frame中 
    if(first_point_cloud_) {
        first_point_cloud_ = false;
        map_ += *point_cloud;
        submap_.push_back(point_cloud);
        ndt_.setInputTarget(point_cloud);

        KeyFrame::Ptr key_frame(new KeyFrame());
        key_frame->setId(key_frames_.size());

        std::cout<<"first key_frame, ID: "<<key_frames_.size()<<std::endl;

        key_frame->setPose(pose_);
        key_frame->setCloud(point_cloud);
        key_frames_.push_back(key_frame);

        std::cout << "\033[1m\033[32m" << "------ Insert keyframe " << key_frames_.size() << " ------" << std::endl;
        return;
    }

    /************************************/
    /****** 1. ndt算法得到大概位姿 *******/
    /************************************/
    // 将点云进行降采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    downSample(point_cloud, sampled_cloud);
    
    // 通过ndt算法，得到变换矩阵，作为优化的起始位姿
    pcl::PointCloud<pcl::PointXYZI> output_cloud;
    ndt_.setInputSource(sampled_cloud);
    ndt_.align(output_cloud, pose_);

    // pose_是Eigen::Matrix4f
    pose_ = ndt_.getFinalTransformation();

    bool converged = ndt_.hasConverged();
    double fitness_score = ndt_.getFitnessScore();
    int final_num_iteration = ndt_.getFinalNumIteration();

    if(!converged) {
        ROS_WARN("NDT does not converge!!!");
    }

    float delta = sqrt(square(pose_(0, 3) - last_update_pose_(0, 3)) +
                       square(pose_(1, 3) - last_update_pose_(1, 3)));

    /************************************/
    /********* 2. 进行位姿图优化 *********/
    /************************************/
    // 关键帧通过距离选取
    if(delta > map_update_distance_) {
        last_update_pose_ = pose_;

        KeyFrame::Ptr key_frame(new KeyFrame());
        key_frame->setId(key_frames_.size());
        key_frame->setPose(pose_);
        //Eigen::Matrix4f转Eigen::Vertex3d和Eigen::Quaterniond
        key_frame->setCloud(point_cloud);
        key_frames_.push_back(key_frame);

        std::cout << "\033[1m\033[32m" << "------ Insert keyframe " << key_frames_.size() << " ------" << std::endl;

        // 检测是否回环
        detectLoopClosure(key_frame);

        // 进行优化
        if(enable_optimize_ && needOptimize()) {
            doPoseOptimize();
        }
        // 添加局部地图，如果局部地图序列长度大于submap_size_，将最早的点云删除
        else {
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(*point_cloud, *transformed_cloud, pose_);
            submap_.push_back(transformed_cloud);
            // 当局部地图达到一定程度，将最早的点云删掉
            while(submap_.size() > submap_size_) {
                submap_.erase(submap_.begin());
            }
            
            // 保证线程安全
            std::unique_lock<std::mutex> locker(map_mutex_);
            map_ += *transformed_cloud;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        for(int i = 0; i < submap_.size(); ++i) {
            *target_cloud += *submap_[i];
        }
        // 更新用于匹配的局部地图
        ndt_.setInputTarget(target_cloud);
    }

    auto t2 = std::chrono::steady_clock::now();
    auto delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    std::cout << "-------------------------------------" << std::endl;
    std::cout << "Sequence number: " << sequence_num_ << std::endl;
    std::cout << "Map size: " << map_.size() << " points." << std::endl;
    std::cout << "Fitness score: " << fitness_score << std::endl;
    std::cout << "Cost time: " << delta_t.count() * 1000.0 << "ms." << std::endl;
    std::cout << "-------------------------------------" << std::endl;
}

// 更新地图
void MapBuilder::updateMap()
{
    std::cout << "Start update map ..." << std::endl;
    std::unique_lock<std::mutex> lock(map_mutex_);
    // 将地图和局部滴入全部清空，重新添加
    map_.clear(); // 点云类型
    submap_.clear(); // 向量类型

    int n = key_frames_.size();
    for(int i = 0; i < n; ++i) {
        // 向地图添加优化后的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*(key_frames_[i]->getCloud()), *transformed_cloud, key_frames_[i]->getPose());
        map_ += *transformed_cloud;

        // 向局部地图添加最新的submap_size_个点云
        if(i > n - submap_size_) {
            submap_.push_back(transformed_cloud);
        }
    }
    std::cout << "Finish update map." << std::endl;
}

// 进行位姿优化
// 1. 首先将相邻两帧keyframe的位姿添加到约束中，直接使用for循环遍历key_frames_
// 2. 然后将回环检测添加到约束中，使用for循环遍历mapofloop
void MapBuilder::doPoseOptimize()
{   
    ceres::Problem problem; // 创建ceres优化问题
    ceres::Solver::Options options; // ceres求解器
    ceres::Solver::Summary summary;
    ceres::LocalParameterization* quaternion_local_parameterization = new ceres::QuaternionParameterization();
    // 初始化ceres优化
    options.max_num_iterations = 1000;
    // options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;

    int kf_len = key_frames_.size(); // keyframe长度，用来生成优化变量
    Eigen::Vector3d array_t[kf_len]; // 存储平移
    Eigen::Quaterniond array_q[kf_len]; // 存储旋转四元数
    array_t[0] = key_frames_[0]->getPosePQ().p;
    array_q[0] = key_frames_[0]->getPosePQ().q;

    // 依次添加key_frames_中的位姿
    for(int i = 1;i<kf_len-1;i++){

        Eigen::Quaterniond source_q, target_q;
        Eigen::Vector3d source_p, target_p;

        // 存储平移和旋转，传值
        array_t[i] = key_frames_[i]->getPosePQ().p;
        array_q[i] = key_frames_[i]->getPosePQ().q;

        // 观测，不需要改变，因此不需要存储
        Eigen::Matrix4f source_pose = key_frames_[i]->getPose();
        Eigen::Matrix4f target_pose = key_frames_[i-1]->getPose();
        source_p = source_pose.block<3,1>(0,3).template cast<double>();
        target_p = target_pose.block<3,1>(0,3).template cast<double>();
        source_q = Eigen::Quaterniond(source_pose.block<3,3>(0,0).template cast<double>());
        target_q = Eigen::Quaterniond(target_pose.block<3,3>(0,0).template cast<double>());

        // 计算pq间的相对运动
        PosePQ relative_measured;
        Eigen::Quaterniond source_q_inv = source_q.conjugate();
        Eigen::Quaterniond relative_q = source_q_inv * target_q;
        Eigen::Vector3d relative_p = source_q_inv*(source_p - target_p);
        relative_measured.p = relative_p;
        relative_measured.q = relative_q;

        ceres::LossFunction* loss_function = NULL;
        ceres::CostFunction* cost_function = PoseGraph3dErrorTerm::Create(relative_measured);

        problem.AddResidualBlock(cost_function, loss_function, 
            array_t[i].data(), array_q[i].coeffs().data(), array_t[i-1].data(), array_q[i-1].coeffs().data());

        // 自定义相加操作
        problem.SetParameterization(array_q[i].coeffs().data(),
                                 quaternion_local_parameterization);
    }

    //将第一个位姿设置为固定，不优化
    problem.SetParameterBlockConstant(array_t[0].data()); 
    problem.SetParameterBlockConstant(array_q[0].coeffs().data());
    problem.SetParameterization(array_q[0].coeffs().data(),
                                 quaternion_local_parameterization);
    
    // 依次添加回环中的位姿约束
    int loop_size = vLoopConstraint.size();
    for(int i = 0; i<loop_size; i++){
        Eigen::Quaterniond source_q, target_q;
        Eigen::Vector3d source_p, target_p;

        // 观测，不需要改变，因此不需要存储
        Eigen::Matrix4f source_pose = vLoopConstraint[i].loop_pose_; // 这里作为观测出现
        Eigen::Matrix4f target_pose = key_frames_[vLoopConstraint[i].second_]->getPose();
        source_p = source_pose.block<3,1>(0,3).template cast<double>();
        target_p = target_pose.block<3,1>(0,3).template cast<double>();
        source_q = Eigen::Quaterniond(source_pose.block<3,3>(0,0).template cast<double>());
        target_q = Eigen::Quaterniond(target_pose.block<3,3>(0,0).template cast<double>());

        // 计算pq间的相对运动
        PosePQ relative_measured;
        Eigen::Quaterniond source_q_inv = source_q.conjugate();
        Eigen::Quaterniond relative_q = source_q_inv * target_q;
        Eigen::Vector3d relative_p = source_q_inv*(source_p - target_p);
        relative_measured.p = relative_p;
        relative_measured.q = relative_q;

        ceres::LossFunction* loss_function = NULL;
        ceres::CostFunction* cost_function = PoseGraph3dErrorTerm::Create(relative_measured);

        problem.AddResidualBlock(cost_function, loss_function, 
            array_t[vLoopConstraint[i].first_].data(), array_q[vLoopConstraint[i].first_].coeffs().data(), 
                            array_t[vLoopConstraint[i].second_].data(), array_q[vLoopConstraint[i].second_].coeffs().data());

    }

    std::cout<<"\033[33mOptimizing....\033[0m"<<std::endl;
    ceres::Solve(options, &problem, &summary);
    
    if(!summary.IsSolutionUsable())
        std::cout << "CERES SOLVE FAILED" <<std::endl;
    else{ // 优化成功，将优化的位姿取出，付给key_frames
        for(int i = 0; i<key_frames_.size(); i++){
            std::cout<< summary.BriefReport() <<std::endl;
            key_frames_[i]->setPosePQ(array_t[i],array_q[i]);
        }
    }
}


// 获取位姿节点，用于可视化
void MapBuilder::getPoseGraph(std::vector<Eigen::Vector3d>& nodes,
                              std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges)
{
    /***************************************************************
    for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_.vertices().begin(); it != optimizer_.vertices().end(); ++it) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(it->second);
        Eigen::Vector3d pt = v->estimate().translation();
        nodes.push_back(pt);
    }

   for(g2o::SparseOptimizer::EdgeSet::iterator it = optimizer_.edges().begin(); it != optimizer_.edges().end(); ++it) {
       g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*>(*it);
       g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(e->vertices()[0]);
       g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(e->vertices()[1]);
       Eigen::Vector3d pt1 = v1->estimate().translation();
       Eigen::Vector3d pt2 = v2->estimate().translation();
       edges.push_back(std::make_pair(pt1, pt2));
   }
   ************************************************************/
}

} // namespace lidar_slam_3d
