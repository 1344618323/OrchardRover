//
// Created by cxn on 2020/7/9.
//

#include "teb_optimal.h"

namespace or_local_planner {
    TebOptimal::TebOptimal() : obstacles_(NULL), visualization_(NULL), via_points_(NULL), cost_(HUGE_VAL),
                               prefer_rotdir_(RotType::NONE), robot_model_(new PointRobotFootprint()),
                               initialized_(false), optimized_(false) {
    }

    TebOptimal::TebOptimal(const Config &config_param,
                           ObstContainer *obstacles,
                           RobotFootprintModelPtr robot_model,
                           LocalVisualizationPtr visual,
                           const ViaPointContainer *via_points) {
        initialize(config_param, obstacles, robot_model, visual, via_points);
    }

    void TebOptimal::initialize(const Config &config_param,
                                ObstContainer *obstacles,
                                RobotFootprintModelPtr robot_model,
                                LocalVisualizationPtr visual,
                                const ViaPointContainer *via_points) {

        optimizer_ = InitOptimizer();

        obstacles_ = obstacles;
        robot_model_ = robot_model;
        visualization_ = visual;
        via_points_ = via_points;
        cost_ = HUGE_VAL;

        vel_start_.first = true;
        vel_start_.second.linear.x = 0;
        vel_start_.second.linear.y = 0;
        vel_start_.second.angular.z = 0;

        vel_end_.first = true;
        vel_end_.second.linear.x = 0;
        vel_end_.second.linear.y = 0;
        vel_end_.second.angular.z = 0;

        param_config_ = config_param;
        robot_info_ = config_param.kinematics_opt();
        obstacles_info_ = config_param.obstacles_opt();
        trajectory_info_ = config_param.trajectory_opt();
        optimization_info_ = config_param.optimize_info();

        initialized_ = true;
    }

    void TebOptimal::RegisterG2OTypes() {
        g2o::Factory *factory = g2o::Factory::instance();
        factory->registerType("TEB_VERTEX_POSE", new g2o::HyperGraphElementCreator<TebVertexPose>);
        factory->registerType("teb_vertex_timediff", new g2o::HyperGraphElementCreator<TebVertexTimeDiff>);

        factory->registerType("TIME_OPTIMAL_EDGE", new g2o::HyperGraphElementCreator<TimeOptimalEdge>);
        factory->registerType("VELOCITY_EDGE", new g2o::HyperGraphElementCreator<VelocityEdge>);
        factory->registerType("VELOCITY_HOLONOMIC_EDGE", new g2o::HyperGraphElementCreator<VelocityHolonomicEdge>);
        factory->registerType("ACCELERATION_EDGE", new g2o::HyperGraphElementCreator<AccelerationEdge>);
        factory->registerType("ACCELERATION_START_EDGE", new g2o::HyperGraphElementCreator<AccelerationStartEdge>);
        factory->registerType("ACCELERATION_GOAL_EDGE", new g2o::HyperGraphElementCreator<AccelerationGoalEdge>);
        factory->registerType("ACCELERATION_HOLONOMIC_EDGE",
                              new g2o::HyperGraphElementCreator<AccelerationHolonomicEdge>);
        factory->registerType("ACCELERATION_HOLONOMIC_START_EDGE",
                              new g2o::HyperGraphElementCreator<AccelerationHolonomicStartEdge>);
        factory->registerType("ACCELERATION_HOLONOMIC_GOAL_EDGE",
                              new g2o::HyperGraphElementCreator<AccelerationHolonomicGoalEdge>);
        factory->registerType("KINEMATICS_DIFF_DRIVE_EDGE", new g2o::HyperGraphElementCreator<KinematicsDiffDriveEdge>);
        factory->registerType("KINEMATICS_CARLIKE_EDGE", new g2o::HyperGraphElementCreator<KinematicsCarlikeEdge>);
        factory->registerType("OBSTACLE_EDGE", new g2o::HyperGraphElementCreator<ObstacleEdge>);
        factory->registerType("VIA_POINT_EDGE", new g2o::HyperGraphElementCreator<ViaPointEdge>);
        factory->registerType("PREFER_ROTDIR_EDGE", new g2o::HyperGraphElementCreator<PreferRotDirEdge>);
        return;
    }

    std::shared_ptr<g2o::SparseOptimizer> TebOptimal::InitOptimizer() {

        static boost::once_flag flag = BOOST_ONCE_INIT;
        boost::call_once(&RegisterG2OTypes, flag);

        std::shared_ptr<g2o::SparseOptimizer> optimizer = std::make_shared<g2o::SparseOptimizer>();
        TebLinearSolver *linearSolver = new TebLinearSolver();
        linearSolver->setBlockOrdering(true);
        TebBlockSolver *blockSolver = new TebBlockSolver(linearSolver);
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

        optimizer->setAlgorithm(solver);

        optimizer->initMultiThreading();

        return optimizer;
    }

    bool TebOptimal::OptimizeTeb(int iterations_innerloop,
                                 int iterations_outerloop,
                                 bool compute_cost_afterwards,
                                 double obst_cost_scale,
                                 double viapoint_cost_scale,
                                 bool alternative_time_cost) {
        if (optimization_info_.optimization_activate() == false) {
            return false;
        }

        bool success = false;
        optimized_ = false;

        double weight_multiplier = 1.0;

        for (int i = 0; i < iterations_outerloop; ++i) {
            if (trajectory_info_.teb_autosize()) {
                //(cxn)根据顶点的时间差，增加或减少顶点
                vertex_console_.AutoResize(trajectory_info_.dt_ref(), trajectory_info_.dt_hysteresis(),
                                           trajectory_info_.min_samples(), trajectory_info_.max_samples());
            }
            //(cxn)给顶点加边
            success = BuildGraph(weight_multiplier);
            if (!success) {
                ClearGraph();
                return false;
            }
            success = OptimizeGraph(iterations_innerloop, false);
            if (!success) {
                ClearGraph();
                return false;
            }
            optimized_ = true;

            if (compute_cost_afterwards && i == iterations_outerloop - 1) {
                ComputeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
            }

            ClearGraph();

            //(cxn)随着迭代的进行，障碍物边的权重会越来越大
            weight_multiplier *= optimization_info_.weight_adapt_factor();
        }

        return true;
    }


    void TebOptimal::Visualize() {
        if (!visualization_) {
            return;
        }

        if (vertex_console_.SizePoses() > 0) {
            visualization_->PublishLocalPlan(vertex_console_);
        }
    }

    void TebOptimal::SetVelocityStart(const geometry_msgs::Twist &vel_start) {
        vel_start_.first = true;
        vel_start_.second.linear.x = vel_start.linear.x;
        vel_start_.second.linear.y = vel_start.linear.y;
        vel_start_.second.angular.z = vel_start.angular.z;
    }

    void TebOptimal::SetVelocityEnd(const geometry_msgs::Twist &vel_end) {
        vel_end_.first = true;
        vel_end_.second = vel_end;
    }

    bool TebOptimal::Optimal(std::vector<DataBase> &initial_plan,
                             const geometry_msgs::Twist *start_vel,
                             bool free_goal_vel, bool micro_control) {
        if (!initialized_) {
            ROS_ERROR("optimal not be initialized");
        }

        /* vertex-console-用于管理Teb中的顶点，若该对象还没有初始化，则初始化
         * 若当前goal与vertex-console-中的goal相差不大，在0.8m以内，则对vertex-console-中的轨迹稍做修改即可，
         *          将vertex-console-的顶点删除到当前start为止，将vertex-console-的终点修改为goal
         * 否则直接重头设置vertex-console-
         */
        if (!vertex_console_.IsInit()) {
            vertex_console_.InitTEBtoGoal(initial_plan, trajectory_info_.dt_ref(),
                                          trajectory_info_.global_plan_overwrite_orientation(),
                                          trajectory_info_.min_samples(),
                                          trajectory_info_.allow_init_with_backwards_motion(),
                                          micro_control);
        } else {
            DataBase start = initial_plan.front();
            DataBase goal = initial_plan.back();

            if ((vertex_console_.SizePoses() > 0) &&
                (goal.GetPosition() - vertex_console_.BackPose().GetPosition()).norm() <
                trajectory_info_.force_reinit_new_goal_dist()) {
                vertex_console_.UpdateAndPruneTEB(start, goal, trajectory_info_.min_samples());
            } else {
                vertex_console_.ClearAllVertex();
                vertex_console_.InitTEBtoGoal(initial_plan, trajectory_info_.dt_ref(),
                                              trajectory_info_.global_plan_overwrite_orientation(),
                                              trajectory_info_.min_samples(),
                                              trajectory_info_.allow_init_with_backwards_motion(),
                                              micro_control);
            }
        }

        if (start_vel) {
            //有没有设置起点速度，如果没有，默认起点速度为0；一般是用里程计实时测算的速度作为起点速度
            //起点速度供 起点的两个pose顶点、一个时间顶点 用加速度边约束
            SetVelocityStart(*start_vel);
        }

        if (free_goal_vel) {
            //到达终点时，速度是否是自由的，如果是自由的，那么终点的两个pose顶点、一个时间顶点就不用加速度边约束了
            //一般我们希望是受控制的（我们设置速度为0）,即free-goal-vel==false，终点的两个pose顶点、一个时间顶点用加速度边约束
            SetVelocityGoalFree();
        } else {
            vel_end_.first = true;
        }

        return OptimizeTeb(optimization_info_.no_inner_iterations(),
                           optimization_info_.no_outer_iterations());
    }

    bool TebOptimal::Optimal(const DataBase &start,
                             const DataBase &goal,
                             const geometry_msgs::Twist *start_vel,
                             bool free_goal_vel, bool micro_control) {
        if (!initialized_) {
            ROS_ERROR("optimal not be initialized");
        }
        if (!vertex_console_.IsInit()) {
            vertex_console_.InitTEBtoGoal(start, goal, 0, 1, trajectory_info_.min_samples(),
                                          trajectory_info_.allow_init_with_backwards_motion());
        } else {
            if (vertex_console_.SizePoses() > 0 &&
                (goal.GetPosition() - vertex_console_.BackPose().GetPosition()).norm() <
                trajectory_info_.force_reinit_new_goal_dist()) {
                vertex_console_.UpdateAndPruneTEB(start, goal, trajectory_info_.min_samples());
            } else {
                vertex_console_.ClearAllVertex();
                vertex_console_.InitTEBtoGoal(start, goal, 0, 1, trajectory_info_.min_samples(),
                                              trajectory_info_.allow_init_with_backwards_motion());
            }
        }
        if (start_vel) {
            SetVelocityStart(*start_vel);
        }

        if (free_goal_vel) {
            SetVelocityGoalFree();
        } else {
            vel_end_.first = true;
        }

        return OptimizeTeb(optimization_info_.no_inner_iterations(),
                           optimization_info_.no_outer_iterations());
    }

    bool TebOptimal::BuildGraph(double weight_multiplier) {
        if (!optimizer_->edges().empty() || !optimizer_->vertices().empty()) {
            ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
            return false;
        }

        AddTebVertices();

        AddObstacleEdges(weight_multiplier);

        //AddViaPointsEdges();

        AddVelocityEdges();

        AddAccelerationEdges();

        AddTimeOptimalEdges();

        if (robot_info_.min_turning_radius() == 0 ||
            optimization_info_.weight_kinematics_turning_radius() == 0) {
            AddKinematicsDiffDriveEdges();
        } else {
            AddKinematicsCarlikeEdges();
        }

        AddPreferRotDirEdges();

        return true;
    }

    bool TebOptimal::OptimizeGraph(int no_iterations, bool clear_after) {
        if (robot_info_.max_vel_x() < 0.01) {
            ROS_WARN("Robot Max Velocity is smaller than 0.01m/s");
            if (clear_after) {
                ClearGraph();
            }
            return false;
        }

        if (!vertex_console_.IsInit() || vertex_console_.SizePoses() < trajectory_info_.min_samples()) {
            ROS_WARN("teb size is too small");
            if (clear_after) {
                ClearGraph();
            }
            return false;
        }

        optimizer_->setVerbose(optimization_info_.optimization_verbose());
        optimizer_->initializeOptimization();

        int iter = optimizer_->optimize(no_iterations);
        if (!iter) {
            ROS_ERROR("optimize failed");
            return false;
        }
        if (clear_after) {
            ClearGraph();
        }

        return true;
    }

    void TebOptimal::ClearGraph() {
        optimizer_->vertices().clear();
        optimizer_->clear();
    }

    void TebOptimal::AddTebVertices() {
        unsigned int id_counter = 0;
        for (int i = 0; i < vertex_console_.SizePoses(); ++i) {
            vertex_console_.PoseVertex(i)->setId(id_counter++);
            optimizer_->addVertex(vertex_console_.PoseVertex(i));
            if (vertex_console_.SizeTimeDiffs() != 0 && i < vertex_console_.SizeTimeDiffs()) {
                vertex_console_.TimeDiffVertex(i)->setId(id_counter++);
                optimizer_->addVertex(vertex_console_.TimeDiffVertex(i));
            }
        }
    }

    /*(cxn)
    给图加障碍物边
    有两种：找到距离机器人左障碍物、右障碍物，加边
           找到距离机器人较近的障碍物，即小于 obstacles-info-.min-obstacle-dist() * obstacles-info-.obstacle-association-force-inclusion-factor()
                                  的障碍物，加边
    */
    void TebOptimal::AddObstacleEdges(double weight_multiplier) {
        if (optimization_info_.weight_obstacle() == 0 || weight_multiplier == 0 || obstacles_ == nullptr) {
            return;
        }

        Eigen::Matrix<double, 1, 1> information;
        information.fill(optimization_info_.weight_obstacle() * weight_multiplier);

        for (int i = 1; i < vertex_console_.SizePoses() - 1; ++i) {
            double left_min_dist = std::numeric_limits<double>::max();
            double right_min_dist = std::numeric_limits<double>::max();
            Obstacle *left_obstacle = nullptr;
            Obstacle *right_obstacle = nullptr;

            std::vector<Obstacle *> relevant_obstacles;

            const Eigen::Vector2d pose_orient = vertex_console_.Pose(i).OrientationUnitVec();

            for (const ObstaclePtr &obst : *obstacles_) {
                double dist = obst->GetMinimumDistance(vertex_console_.Pose(i).GetPosition());
                if (dist < obstacles_info_.min_obstacle_dist() *
                           obstacles_info_.obstacle_association_force_inclusion_factor()) {
                    relevant_obstacles.push_back(obst.get());
                    continue;
                }
                if (dist > obstacles_info_.min_obstacle_dist() *
                           obstacles_info_.obstacle_association_cutoff_factor()) {
                    continue;
                }

                //看叉积正负，就是看sin的正负，若是则说明障碍物在车的左边
                //注意我们用的是点障碍物，所以获取障碍物质心，其实就是获取其坐标
                if (Cross2D(pose_orient, obst->GetCentroid()) > 0) {
                    if (dist < left_min_dist) {
                        left_min_dist = dist;
                        left_obstacle = obst.get();
                    }
                } else {
                    if (dist < right_min_dist) {
                        right_min_dist = dist;
                        right_obstacle = obst.get();
                    }
                }
            }

            if (left_obstacle) {

                ObstacleEdge *dist_bandpt_obst = new ObstacleEdge;
                dist_bandpt_obst->setVertex(0, vertex_console_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information);
                dist_bandpt_obst->SetParameters(param_config_, robot_model_.get(), left_obstacle);
                optimizer_->addEdge(dist_bandpt_obst);
            }

            if (right_obstacle) {

                ObstacleEdge *dist_bandpt_obst = new ObstacleEdge;
                dist_bandpt_obst->setVertex(0, vertex_console_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information);
                dist_bandpt_obst->SetParameters(param_config_, robot_model_.get(), right_obstacle);
                optimizer_->addEdge(dist_bandpt_obst);
            }

            for (const Obstacle *obst : relevant_obstacles) {

                ObstacleEdge *dist_bandpt_obst = new ObstacleEdge;
                dist_bandpt_obst->setVertex(0, vertex_console_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information);
                dist_bandpt_obst->SetParameters(param_config_, robot_model_.get(), obst);
                optimizer_->addEdge(dist_bandpt_obst);
            }
        }
    }

    void TebOptimal::AddViaPointsEdges() {
        if (optimization_info_.weight_viapoint() == 0 || via_points_ == NULL || via_points_->empty()) {
            return;
        }
        int start_pose_idx = 0;
        int n = vertex_console_.SizePoses();
        if (n < 3) {
            return;
        }

        for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it) {

            int index = vertex_console_.FindClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
            if (trajectory_info_.via_points_ordered()) {
                start_pose_idx = index + 2;
            }

            if (index > n - 2) {
                index = n - 2;
            }

            if (index < 1) {
                index = 1;
            }

            Eigen::Matrix<double, 1, 1> information;
            information.fill(optimization_info_.weight_viapoint());

            ViaPointEdge *edge_viapoint = new ViaPointEdge;
            edge_viapoint->setVertex(0, vertex_console_.PoseVertex(index));
            edge_viapoint->setInformation(information);
            edge_viapoint->SetParameters(&(*vp_it));
            optimizer_->addEdge(edge_viapoint);
        }
    }

    //根据参数中是不是有 max-vel-y，来判断其是 非完整性运动约束 还是完整性运动约束
    void TebOptimal::AddVelocityEdges() {
        if (robot_info_.max_vel_y() == 0) {
            if (optimization_info_.weight_max_vel_x() == 0 && optimization_info_.weight_max_vel_theta() == 0) {
                return;
            }

            int n = vertex_console_.SizePoses();
            Eigen::Matrix<double, 2, 2> information;
            information(0, 0) = optimization_info_.weight_max_vel_x();
            information(1, 1) = optimization_info_.weight_max_vel_theta();
            information(0, 1) = 0.0;
            information(1, 0) = 0.0;

            for (int i = 0; i < n - 1; ++i) {
                VelocityEdge *velocity_edge = new VelocityEdge;
                velocity_edge->setVertex(0, vertex_console_.PoseVertex(i));
                velocity_edge->setVertex(1, vertex_console_.PoseVertex(i + 1));
                velocity_edge->setVertex(2, vertex_console_.TimeDiffVertex(i));
                velocity_edge->setInformation(information);
                velocity_edge->SetConfig(param_config_);
                optimizer_->addEdge(velocity_edge);
            }
        } else {
            if (optimization_info_.weight_max_vel_x() == 0 && optimization_info_.weight_max_vel_y() == 0 &&
                optimization_info_.weight_max_vel_theta() == 0) {
                return;
            }

            int n = vertex_console_.SizePoses();
            Eigen::Matrix<double, 3, 3> information;
            information.fill(0);
            information(0, 0) = optimization_info_.weight_max_vel_x();
            information(1, 1) = optimization_info_.weight_max_vel_y();
            information(2, 2) = optimization_info_.weight_max_vel_theta();

            for (int i = 0; i < n - 1; ++i) {
                VelocityHolonomicEdge *velocity_edge = new VelocityHolonomicEdge;
                velocity_edge->setVertex(0, vertex_console_.PoseVertex(i));
                velocity_edge->setVertex(1, vertex_console_.PoseVertex(i + 1));
                velocity_edge->setVertex(2, vertex_console_.TimeDiffVertex(i));
                velocity_edge->setInformation(information);
                velocity_edge->SetConfig(param_config_);
                optimizer_->addEdge(velocity_edge);
            }
        }
    }

    void TebOptimal::AddAccelerationEdges() {
        if (optimization_info_.weight_acc_lim_x() == 0 && optimization_info_.weight_acc_lim_theta() == 0) {
            return;
        }

        int n = vertex_console_.SizePoses();

        if (robot_info_.max_vel_y() == 0 || robot_info_.acc_lim_y() == 0) {
            Eigen::Matrix<double, 2, 2> information;
            information.fill(0);
            information(0, 0) = optimization_info_.weight_acc_lim_x();
            information(1, 1) = optimization_info_.weight_acc_lim_theta();

            if (vel_start_.first) {
                AccelerationStartEdge *acceleration_edge = new AccelerationStartEdge;
                acceleration_edge->setVertex(0, vertex_console_.PoseVertex(0));
                acceleration_edge->setVertex(1, vertex_console_.PoseVertex(1));
                acceleration_edge->setVertex(2, vertex_console_.TimeDiffVertex(0));
                acceleration_edge->SetInitialVelocity(vel_start_.second);
                acceleration_edge->setInformation(information);
                acceleration_edge->SetConfig(param_config_);
                optimizer_->addEdge(acceleration_edge);
            }

            for (int i = 0; i < n - 2; ++i) {
                AccelerationEdge *acceleration_edge = new AccelerationEdge;
                acceleration_edge->setVertex(0, vertex_console_.PoseVertex(i));
                acceleration_edge->setVertex(1, vertex_console_.PoseVertex(i + 1));
                acceleration_edge->setVertex(2, vertex_console_.PoseVertex(i + 2));
                acceleration_edge->setVertex(3, vertex_console_.TimeDiffVertex(i));
                acceleration_edge->setVertex(4, vertex_console_.TimeDiffVertex(i + 1));
                acceleration_edge->setInformation(information);
                acceleration_edge->SetConfig(param_config_);
                optimizer_->addEdge(acceleration_edge);
            }

            if (vel_end_.first) {
                AccelerationGoalEdge *acceleration_edge = new AccelerationGoalEdge;
                acceleration_edge->setVertex(0, vertex_console_.PoseVertex(n - 2));
                acceleration_edge->setVertex(1, vertex_console_.PoseVertex(n - 1));
                acceleration_edge->setVertex(2, vertex_console_.TimeDiffVertex(vertex_console_.SizeTimeDiffs() - 1));
                acceleration_edge->SetGoalVelocity(vel_end_.second);
                acceleration_edge->setInformation(information);
                acceleration_edge->SetConfig(param_config_);
                optimizer_->addEdge(acceleration_edge);
            }
        } else {
            Eigen::Matrix<double, 3, 3> information;
            information.fill(0);
            information(0, 0) = optimization_info_.weight_acc_lim_x();
            information(1, 1) = optimization_info_.weight_acc_lim_y();
            information(2, 2) = optimization_info_.weight_acc_lim_theta();

            if (vel_start_.first) {
                AccelerationHolonomicStartEdge *acceleration_edge = new AccelerationHolonomicStartEdge;
                acceleration_edge->setVertex(0, vertex_console_.PoseVertex(0));
                acceleration_edge->setVertex(1, vertex_console_.PoseVertex(1));
                acceleration_edge->setVertex(2, vertex_console_.TimeDiffVertex(0));
                acceleration_edge->setInitialVelocity(vel_start_.second);
                acceleration_edge->setInformation(information);
                acceleration_edge->SetConfig(param_config_);
                optimizer_->addEdge(acceleration_edge);
            }

            for (int i = 0; i < n - 2; ++i) {
                AccelerationHolonomicEdge *acceleration_edge = new AccelerationHolonomicEdge;
                acceleration_edge->setVertex(0, vertex_console_.PoseVertex(i));
                acceleration_edge->setVertex(1, vertex_console_.PoseVertex(i + 1));
                acceleration_edge->setVertex(2, vertex_console_.PoseVertex(i + 2));
                acceleration_edge->setVertex(3, vertex_console_.TimeDiffVertex(i));
                acceleration_edge->setVertex(4, vertex_console_.TimeDiffVertex(i + 1));
                acceleration_edge->setInformation(information);
                acceleration_edge->SetConfig(param_config_);
                optimizer_->addEdge(acceleration_edge);
            }

            if (vel_end_.first) {
                AccelerationHolonomicGoalEdge *acceleration_edge = new AccelerationHolonomicGoalEdge;
                acceleration_edge->setVertex(0, vertex_console_.PoseVertex(n - 2));
                acceleration_edge->setVertex(1, vertex_console_.PoseVertex(n - 1));
                acceleration_edge->setVertex(2, vertex_console_.TimeDiffVertex(vertex_console_.SizeTimeDiffs() - 1));
                acceleration_edge->SetGoalVelocity(vel_end_.second);
                acceleration_edge->setInformation(information);
                acceleration_edge->SetConfig(param_config_);
                optimizer_->addEdge(acceleration_edge);
            }
        }
    }

    void TebOptimal::AddTimeOptimalEdges() {
        if (optimization_info_.weight_optimaltime() == 0) {
            return;
        }

        Eigen::Matrix<double, 1, 1> information;
        information.fill(optimization_info_.weight_optimaltime());

        for (int i = 0; i < vertex_console_.SizeTimeDiffs(); ++i) {
            TimeOptimalEdge *timeoptimal_edge = new TimeOptimalEdge;
            timeoptimal_edge->setVertex(0, vertex_console_.TimeDiffVertex(i));
            timeoptimal_edge->setInformation(information);
            timeoptimal_edge->SetConfig(param_config_);
            optimizer_->addEdge(timeoptimal_edge);
        }
    }

    void TebOptimal::AddKinematicsDiffDriveEdges() {
        if (optimization_info_.weight_kinematics_nh() == 0 &&
            optimization_info_.weight_kinematics_forward_drive() == 0) {
            return;
        }

        Eigen::Matrix<double, 2, 2> information_kinematics;
        information_kinematics.fill(0.0);
        information_kinematics(0, 0) = optimization_info_.weight_kinematics_nh();
        information_kinematics(1, 1) = optimization_info_.weight_kinematics_forward_drive();

        for (int i = 0; i < vertex_console_.SizePoses() - 1; i++) {
            KinematicsDiffDriveEdge *kinematics_edge = new KinematicsDiffDriveEdge;
            kinematics_edge->setVertex(0, vertex_console_.PoseVertex(i));
            kinematics_edge->setVertex(1, vertex_console_.PoseVertex(i + 1));
            kinematics_edge->setInformation(information_kinematics);
            kinematics_edge->SetConfig(param_config_);
            optimizer_->addEdge(kinematics_edge);
        }
    }

    void TebOptimal::AddKinematicsCarlikeEdges() {
        if (optimization_info_.weight_kinematics_nh() == 0 && optimization_info_.weight_kinematics_turning_radius()) {
            return;
        }

        Eigen::Matrix<double, 2, 2> information_kinematics;
        information_kinematics.fill(0.0);
        information_kinematics(0, 0) = optimization_info_.weight_kinematics_nh();
        information_kinematics(1, 1) = optimization_info_.weight_kinematics_turning_radius();

        for (int i = 0; i < vertex_console_.SizePoses() - 1; i++) {
            KinematicsCarlikeEdge *kinematics_edge = new KinematicsCarlikeEdge;
            kinematics_edge->setVertex(0, vertex_console_.PoseVertex(i));
            kinematics_edge->setVertex(1, vertex_console_.PoseVertex(i + 1));
            kinematics_edge->setInformation(information_kinematics);
            kinematics_edge->SetConfig(param_config_);
            optimizer_->addEdge(kinematics_edge);
        }
    }

    void TebOptimal::AddPreferRotDirEdges() {
        if (prefer_rotdir_ == RotType::NONE || optimization_info_.weight_prefer_rotdir() == 0) {
            return;
        }

        Eigen::Matrix<double, 1, 1> information_rotdir;
        information_rotdir.fill(optimization_info_.weight_prefer_rotdir());

        for (int i = 0; i < vertex_console_.SizePoses() - 1 && i < 3; ++i) {
            PreferRotDirEdge *rotdir_edge = new PreferRotDirEdge;
            rotdir_edge->setVertex(0, vertex_console_.PoseVertex(i));
            rotdir_edge->setVertex(1, vertex_console_.PoseVertex(i + 1));
            rotdir_edge->setInformation(information_rotdir);

            if (prefer_rotdir_ == RotType::LEFT) {
                rotdir_edge->PreferLeft();
            } else {
                rotdir_edge->PreferRight();
            }
            optimizer_->addEdge(rotdir_edge);
        }
    }

    void
    TebOptimal::ComputeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost) {
        bool graph_exist_flag(false);
        if (optimizer_->edges().empty() && optimizer_->vertices().empty()) {
            BuildGraph();
            optimizer_->initializeOptimization();
        } else {
            graph_exist_flag = true;
        }

        optimizer_->computeInitialGuess();

        cost_ = 0;

        if (alternative_time_cost) {
            cost_ += vertex_console_.GetSumOfAllTimeDiffs();
        }

        for (std::vector<g2o::OptimizableGraph::Edge *>::const_iterator it = optimizer_->activeEdges().begin();
             it != optimizer_->activeEdges().end(); it++) {
            TimeOptimalEdge *edge_time_optimal = dynamic_cast<TimeOptimalEdge *>(*it);
            if (edge_time_optimal != NULL && !alternative_time_cost) {
                cost_ += edge_time_optimal->GetError().squaredNorm();
                continue;
            }

            KinematicsDiffDriveEdge *edge_kinematics_dd = dynamic_cast<KinematicsDiffDriveEdge *>(*it);
            if (edge_kinematics_dd != NULL) {
                cost_ += edge_kinematics_dd->GetError().squaredNorm();
                continue;
            }

            KinematicsCarlikeEdge *edge_kinematics_cl = dynamic_cast<KinematicsCarlikeEdge *>(*it);
            if (edge_kinematics_cl != NULL) {
                cost_ += edge_kinematics_cl->GetError().squaredNorm();
                continue;
            }

            VelocityEdge *edge_velocity = dynamic_cast<VelocityEdge *>(*it);
            if (edge_velocity != NULL) {
                cost_ += edge_velocity->GetError().squaredNorm();
                continue;
            }

            AccelerationEdge *edge_acceleration = dynamic_cast<AccelerationEdge *>(*it);
            if (edge_acceleration != NULL) {
                cost_ += edge_acceleration->GetError().squaredNorm();
                continue;
            }

            ObstacleEdge *edge_obstacle = dynamic_cast<ObstacleEdge *>(*it);
            if (edge_obstacle != NULL) {
                cost_ += edge_obstacle->GetError().squaredNorm() * obst_cost_scale;
                continue;
            }

            ViaPointEdge *edge_viapoint = dynamic_cast<ViaPointEdge *>(*it);
            if (edge_viapoint != NULL) {
                cost_ += edge_viapoint->GetError().squaredNorm() * viapoint_cost_scale;
                continue;
            }
        }

        if (!graph_exist_flag) {
            ClearGraph();
        }
    }

    //只看轨迹点前几个是不是满足要求，会根据机器人模型找到模型中每个点在costmap中的最大map，若cost值<0，我们就认为轨迹不能用
    bool TebOptimal::IsTrajectoryFeasible(or_common::ErrorInfo &error_info, RobotPositionCost *position_cost,
                                          const std::vector<Eigen::Vector2d> &footprint_spec,
                                          double inscribed_radius, double circumscribed_radius, int look_ahead_idx) {

        if (look_ahead_idx < 0 || look_ahead_idx >= vertex_console_.SizePoses()) {
            look_ahead_idx = vertex_console_.SizePoses() - 1;
        }

        for (int i = 0; i <= look_ahead_idx; ++i) {
            auto position = vertex_console_.Pose(i).GetPosition();
            if (position_cost->FootprintCost(position.coeffRef(0), position.coeffRef(1),
                                             vertex_console_.Pose(i).GetTheta(), footprint_spec,
                                             inscribed_radius, circumscribed_radius) < 0) {
                return false;
            }

            if (i < look_ahead_idx) {
                auto next_position = vertex_console_.Pose(i + 1).GetPosition();
                if ((next_position - position).norm() > inscribed_radius) {
                    DataBase center;
                    center.AverageInPlace(vertex_console_.Pose(i), vertex_console_.Pose(i + 1));
                    if (position_cost->FootprintCost(center.GetPosition().coeffRef(0), center.GetPosition().coeffRef(1),
                                                     center.GetTheta(), footprint_spec,
                                                     inscribed_radius, circumscribed_radius) < 0) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    bool TebOptimal::GetVelocity(or_common::ErrorInfo &error_info, double &vx, double &vy, double &omega,
                                 double &acc_x, double &acc_y, double &acc_omega) const {

        if (vertex_console_.SizePoses() < 2) {
            ROS_ERROR("pose is too less to compute the velocity");
            vx = 0;
            vy = 0;
            omega = 0;
            return false;
        }

        double dt = vertex_console_.TimeDiff(0);
        if (dt <= 0) {
            ROS_ERROR("the time between two pose is nagetive");
            vx = 0;
            vy = 0;
            omega = 0;
            return false;
        }

        ExtractVelocity(vertex_console_.Pose(0), vertex_console_.Pose(1), dt, vx, vy, omega);

        double dt_2 = vertex_console_.TimeDiff(1);
        double vx_2, vy_2, omega_2;

        ExtractVelocity(vertex_console_.Pose(1), vertex_console_.Pose(2), dt_2, vx_2, vy_2, omega_2);
        acc_x = (vx_2 - vx) / dt;
        acc_y = (vy_2 - vy) / dt;
        acc_omega = (omega_2 - omega) / dt;
        return true;
    }

    void TebOptimal::ExtractVelocity(const DataBase &pose1,
                                     const DataBase &pose2,
                                     double dt,
                                     double &vx,
                                     double &vy,
                                     double &omega) const {
        if (dt == 0) {
            vx = 0;
            vy = 0;
            omega = 0;
            return;
        }

        Eigen::Vector2d deltaS = pose2.GetPosition() - pose1.GetPosition();
        if (robot_info_.max_vel_y() == 0) {
            Eigen::Vector2d conf1dir(cos(pose1.GetTheta()), sin(pose1.GetTheta()));
            double dir = deltaS.dot(conf1dir);
            vx = (double) g2o::sign(dir) * deltaS.norm() / dt;
            vy = 0;
        } else {
            double cos_theta1 = std::cos(pose1.GetTheta());
            double sin_theta1 = std::sin(pose1.GetTheta());
            double p1_dx = cos_theta1 * deltaS.coeffRef(0) + sin_theta1 * deltaS.coeffRef(1);
            double p1_dy = -sin_theta1 * deltaS.coeffRef(0) + cos_theta1 * deltaS.coeffRef(1);
            vx = p1_dx / dt;
            vy = p1_dy / dt;
        }
        double orientdiff = g2o::normalize_theta(pose2.GetTheta() - pose1.GetTheta());
        omega = orientdiff / dt;
    }
} // namespace or_local_planner