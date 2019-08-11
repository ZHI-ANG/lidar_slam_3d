#include "map_builder.h"
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

namespace lidar_slam_3d
{
typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

// 默认构造函数，构造时认为是第一个点云，编号为0
MapBuilder::MapBuilder() :
    first_point_cloud_(true), sequence_num_(0),
    pose_(Eigen::Matrix4f::Identity()), last_update_pose_(Eigen::Matrix4f::Identity()),
    submap_size_(30), voxel_grid_leaf_size_(2.0), map_update_distance_(1.0), enable_optimize_(false),
    loop_search_distance_(20.0), loop_min_chain_size_(5), loop_min_fitness_score_(1.5),
    loop_keyframe_skip_(20), loop_constraint_count_(0), optimize_every_n_constraint_(10)
{
    // ndt算法参数
    ndt_.setTransformationEpsilon(0.01);
    ndt_.setStepSize(0.1);
    ndt_.setResolution(1.0);
    ndt_.setMaximumIterations(30);

    SlamBlockSolver::LinearSolverType* linear_solver = new SlamLinearSolver;
    SlamBlockSolver* solver_ptr = new SlamBlockSolver(linear_solver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // L-M
    optimizer_.setAlgorithm(solver);
    optimizer_.setVerbose(false);
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

// 添加优化图的顶点，每个顶点为关键帧计算出的位姿变换矩阵
void MapBuilder::addVertex(const KeyFrame::Ptr& key_frame)
{
    g2o::VertexSE3* vertex(new g2o::VertexSE3());
    vertex->setId(key_frame->getId());
    // 三维欧式变换矩阵isometrye3d，接收一个matrix4f的矩阵
    vertex->setEstimate(Eigen::Isometry3d(key_frame->getPose().cast<double>()));
    optimizer_.addVertex(vertex);
}

void MapBuilder::addEdge(const KeyFrame::Ptr& source, const Eigen::Matrix4f& source_pose,
                         const KeyFrame::Ptr& target, const Eigen::Matrix4f& target_pose,
                         const Eigen::Matrix<double, 6, 6>& information)
{
    static int edge_count = 0;

    g2o::EdgeSE3* edge = new g2o::EdgeSE3;

    int source_id = source->getId();
    int target_id = target->getId();

    edge->vertices()[0] = optimizer_.vertex(source_id);
    edge->vertices()[1] = optimizer_.vertex(target_id);
    // 上一帧的位姿T_r_w和当前帧的位姿T_c_w的变换关系：
    // 
    Eigen::Isometry3d relative_pose((source_pose.inverse() * target_pose).cast<double>());
    edge->setId(edge_count);
    edge->setMeasurement(relative_pose);
    edge->setInformation(information);
    edge_count++;

    optimizer_.addEdge(edge);
}

// 查找回环检测中最接近当前帧位置的历史帧
// 这里使用两帧之间的位置的二范数作为距离
KeyFrame::Ptr MapBuilder::getClosestKeyFrame(const KeyFrame::Ptr& key_frame,
                                             const std::vector<KeyFrame::Ptr>& candidates)
{
    Eigen::Vector3f pt1 = key_frame->getPose().block<3, 1>(0, 3);
    float min_distance = std::numeric_limits<float>::max();
    int id;

    for(const KeyFrame::Ptr& frame : candidates) {
        Eigen::Vector3f pt2 = frame->getPose().block<3, 1>(0, 3);
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
    Eigen::Vector3f pt1 = key_frame->getPose().block<3, 1>(0, 3);

    // 在历史关键帧中遍历，求出当前帧和历史帧位置的距离
    for(int i = 0; i < n; ++i) {
        Eigen::Vector3f pt2 = key_frames_[i]->getPose().block<3, 1>(0, 3);
        float distance = (pt1 - pt2).norm();

        // 若历史关键帧kf[i]和当前关键帧距离小于回环搜索距离，即可能存在回环
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
        // 若历史关键帧kf[i]和当前关键帧距离大于回环搜索距离，即不需要检测回环
        // 同时回环链的大小大于回环链最小长度，将回环链添加进回环链中，这个主要是为了构建局部地图，更好的找到回环位置
        // 否则，不添加
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
            // 将回环
            addEdge(key_frame, loop_pose,
                    closest_keyframe, closest_keyframe->getPose(),
                    Eigen::Matrix<double, 6, 6>::Identity());
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
        key_frame->setPose(pose_);
        key_frame->setCloud(point_cloud);
        key_frames_.push_back(key_frame);
        addVertex(key_frame);
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
        key_frame->setCloud(point_cloud);
        key_frames_.push_back(key_frame);

        // 添加进优化器中，优化变量为相邻两帧
        addVertex(key_frame);
        addEdge(key_frame, pose_,
                key_frames_[key_frame->getId() - 1], key_frames_[key_frame->getId() - 1]->getPose(),
                Eigen::Matrix<double, 6, 6>::Identity());

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
void MapBuilder::doPoseOptimize()
{
    // 初始点设为固定，不参与优化
    g2o::OptimizableGraph::Vertex* v = optimizer_.vertex(0);
    v->setFixed(true);

    optimizer_.initializeOptimization();

    // 残差
    double chi2 = optimizer_.chi2();
    auto t1 = std::chrono::steady_clock::now();
    int iter = optimizer_.optimize(100);
    auto t2 = std::chrono::steady_clock::now();
    auto delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    if (iter > 0) {
        std::cout << "Optimization finished after " << iter << " iterations. Cost time " <<
                     delta_t.count() * 1000.0 << "ms." << std::endl;
        std::cout << "chi2: (before)" << chi2 << " -> (after)" << optimizer_.chi2() << std::endl;
    }
    else {
        std::cout << "Optimization failed, result might be invalid!" << std::endl;
        return;
    }

    // 从优化后的顶点中取出位姿，付给kf
    for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_.vertices().begin(); it != optimizer_.vertices().end(); ++it) {
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(it->second);
        key_frames_[v->id()]->setPose(v->estimate().matrix().cast<float>());
    }

    optimize_time_ = std::chrono::steady_clock::now();
    loop_constraint_count_ = 0;

    updateMap();

    // vector.back()和front()方法，返回的是vector中的元素引用
    // vector.begin()和end()方法，返回的是vector中第一个和最后一个元素的迭代器
    pose_ = key_frames_.back()->getPose();
}

void MapBuilder::getPoseGraph(std::vector<Eigen::Vector3d>& nodes,
                              std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges)
{
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
}

} // namespace lidar_slam_3d
