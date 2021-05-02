//
// Created by cxn on 2020/7/6.
//
#include "teb_local_planner.h"

namespace or_local_planner {

    TebLocalPlanner::TebLocalPlanner() {
    }

    TebLocalPlanner::~TebLocalPlanner() {
    }

    /*
     * 外部调用函数
     * 获得的是机器人在odom坐标系下的坐标
     * 获取odom测量的vel
     * 计算实时的 Todom-map
     * 若机器人已经到达目标（odom系下），则返回成功
     * 遍历全局路径，直到路径点距当前位点0.8m以内，将之前的路径剔除了
     * 提取一段待优化路径，并将路径转化为odom坐标系
     * 从costamp中搜集障碍物坐标到obst-vector-中
     * 用设定值global-plan-viapoint-sep分割待优化路径，得到一些必须通过的轨迹点
     */
    or_common::ErrorInfo TebLocalPlanner::ComputeVelocityCommands(or_msgs::TwistAccel &cmd_vel) {

        if (!is_initialized_) {
            ROS_ERROR("timed_elastic_band doesn't be initialized");
            or_common::ErrorInfo algorithm_init_error(or_common::LP_ALGORITHM_INITILIZATION_ERROR,
                                                      "teb initialize failed");
            ROS_ERROR("%s", algorithm_init_error.error_msg().c_str());
            return algorithm_init_error;
        }

        GetPlan(temp_plan_);

        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.linear.y = 0;
        cmd_vel.twist.angular.z = 0;

        cmd_vel.accel.linear.x = 0;
        cmd_vel.accel.linear.y = 0;
        cmd_vel.accel.angular.z = 0;

        //(cxn)这里获得的是机器人在odom坐标系下的坐标
        UpdateRobotPose();
        //(cxn)获取odom测量的vel
        // UpdateRobotVel();
        //(cxn)计算实时的 Todom-map
        UpdateGlobalToPlanTranform();

        auto time_now = std::chrono::system_clock::now();
        oscillation_time_ =
                std::chrono::duration_cast<std::chrono::milliseconds>(time_now - oscillation_).count() / 1000.0f;

        if (oscillation_time_ > 1.0) {
            //若距上次调用此函数，已经过了1s，则若车的移动较小，我们就清理下costmap
            if ((robot_pose_.GetPosition() - last_robot_pose_.GetPosition()).norm() < 0.1) {
                local_cost_.lock()->ClearCostMap();
            } else {
                oscillation_time_ = 0;
                oscillation_ = std::chrono::system_clock::now();
                last_robot_pose_ = robot_pose_;
            }
        }

        //若机器人已经到达目标（odom系下），则返回成功
        if (IsGoalReached()) {
            or_common::ErrorInfo algorithm_ok(or_common::OK, "reached the goal");
            ROS_INFO("reached the goal");
            return algorithm_ok;
        }

        //遍历路径，直到路径点距当前位点0.8m以内，将之前的路径剔除了
        PruneGlobalPlan();

        //提取一段待优化路径，并将路径转化为odom坐标系
        int goal_idx;
        if (!TransformGlobalPlan(&goal_idx)) {
            or_common::ErrorInfo PlanTransformError(or_common::LP_PLANTRANSFORM_ERROR, "plan transform error");
            ROS_ERROR("%s", PlanTransformError.error_msg().c_str());
            return PlanTransformError;
        }

        if (transformed_plan_.empty()) {
            or_common::ErrorInfo PlanTransformError(or_common::LP_PLANTRANSFORM_ERROR, "transformed plan is empty");
            ROS_ERROR("transformed plan is empty");
            return PlanTransformError;
        }


        if (global_plan_overwrite_orientation_) {
            transformed_plan_.back().SetTheta(EstimateLocalGoalOrientation(transformed_plan_.back(), goal_idx));
        }

        if (transformed_plan_.size() == 1) {
            // plan only contains the goal
            transformed_plan_.insert(transformed_plan_.begin(), robot_pose_); // insert start (not yet initialized)
        } else {
            transformed_plan_.front() = robot_pose_; // update start;
        }

        //从costamp中搜集障碍物坐标到obst-vector-中
        obst_vector_.clear();
        robot_goal_ = transformed_plan_.back();
        UpdateObstacleWithCostmap(robot_goal_.GetPosition());

        //用设定值global-plan-viapoint-sep分割待优化路径，得到一些必须通过的轨迹点
        UpdateViaPointsContainer();

        bool micro_control = false;
        if (global_plan_.poses.back().pose.position.z == 1) {
            micro_control = true;
        }

        // bool success = optimal_->Optimal(transformed_plan_, &robot_current_vel_, free_goal_vel_, micro_control);
        bool success = optimal_->Optimal(transformed_plan_, nullptr , free_goal_vel_, micro_control);

        if (!success) {
            optimal_->ClearPlanner();
            or_common::ErrorInfo OptimalError(or_common::LP_OPTIMAL_ERROR, "optimal error");
            ROS_ERROR("optimal error");
            last_cmd_ = cmd_vel;
            return OptimalError;
        }

        bool feasible = optimal_->IsTrajectoryFeasible(teb_error_info_, robot_cost_.get(), robot_footprint_,
                                                       robot_inscribed_radius_, robot_circumscribed_radius,
                                                       fesiable_step_look_ahead_);
        if (!feasible) {
            optimal_->ClearPlanner();
            last_cmd_ = cmd_vel;
            ROS_ERROR("trajectory is not feasible");
            or_common::ErrorInfo trajectory_error(or_common::LP_ALGORITHM_TRAJECTORY_ERROR,
                                                  "trajectory is not feasible");
            return trajectory_error;
        }

        if (!optimal_->GetVelocity(teb_error_info_, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y,
                                   cmd_vel.twist.angular.z,
                                   cmd_vel.accel.linear.x, cmd_vel.accel.linear.y, cmd_vel.accel.angular.z)) {
            optimal_->ClearPlanner();
            ROS_ERROR("can not get the velocity");
            or_common::ErrorInfo velocity_error(or_common::LP_VELOCITY_ERROR, "velocity is not feasible");
            last_cmd_ = cmd_vel;
            return velocity_error;
        }

        //对输出做限制，避免输出在最值之外
        SaturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                         max_vel_x_, max_vel_y_, max_vel_theta_, max_vel_x_backwards);

        last_cmd_ = cmd_vel;

        optimal_->Visualize();

        ROS_INFO("compute velocity succeed");
        return or_common::ErrorInfo(or_common::ErrorCode::OK);
    }

    bool TebLocalPlanner::IsGoalReached() {

        tf::Stamped<tf::Pose> global_goal;
        tf::poseStampedMsgToTF(global_plan_.poses.back(), global_goal);
        //(cxn)转成 Podom
        global_goal.setData(plan_to_global_transform_ * global_goal);
        auto goal = DataConverter::LocalConvertTFData(global_goal);

        auto distance = (goal.first - robot_pose_.GetPosition()).norm();
        double delta_orient = g2o::normalize_theta(goal.second - robot_pose_.GetTheta());

        if (distance < xy_goal_tolerance_ && fabs(delta_orient) < yaw_goal_tolerance_) {
            ROS_INFO("goal reached");
            return true;
        } else {
            return false;
        }
    }

    bool TebLocalPlanner::SetPlan(const nav_msgs::Path &plan, const geometry_msgs::PoseStamped &goal) {
        if (plan_mutex_.try_lock()) {
            ROS_INFO("set plan");
            if (plan.poses.empty()) {
                temp_plan_.poses.push_back(goal);
            } else {
                temp_plan_ = plan;
            }
            plan_mutex_.unlock();
        }
    }

    bool TebLocalPlanner::GetPlan(const nav_msgs::Path &plan) {
        if (plan_mutex_.try_lock()) {
            //(cxn)这个变量存的是“map”坐标系下的路径
            global_plan_ = plan;
            plan_mutex_.unlock();
        }
    }

    //(cxn)遍历global-planner给的路径，直到找到距离机器人小于0.8m的waypoint，路径中先前的waypoints都删了
    bool TebLocalPlanner::PruneGlobalPlan() {
        if (global_plan_.poses.empty()) {
            return true;
        }
        try {
            //存储Tmap_odom
            tf::StampedTransform global_to_plan_transform;

            //(cxn)这里获取Tmap-odom，得到机器人位姿 Pmap=Tmap-odom×Podom
            tf_.lock()->lookupTransform(global_plan_.poses.front().header.frame_id,
                                        robot_tf_pose_.frame_id_, ros::Time(0),
                                        global_to_plan_transform);
            tf::Stamped<tf::Pose> robot;
            robot.setData(global_to_plan_transform * robot_tf_pose_);

            //机器人当前坐标到全局路径终点坐标 之差
            Eigen::Vector2d robot_to_goal(robot.getOrigin().x() - global_plan_.poses.back().pose.position.x,
                                          robot.getOrigin().y() - global_plan_.poses.back().pose.position.y);

            for (auto iterator = global_plan_.poses.begin(); iterator != global_plan_.poses.end(); ++iterator) {
                Eigen::Vector2d temp_vector(robot.getOrigin().x() - iterator->pose.position.x,
                                            robot.getOrigin().y() - iterator->pose.position.y);
                if (temp_vector.norm() < 0.8) {
                    if (iterator == global_plan_.poses.begin()) {
                        break;
                    }
                    global_plan_.poses.erase(global_plan_.poses.begin(), iterator);
                    break;
                }
            }
        }
        catch (const tf::TransformException &ex) {
            ROS_ERROR("prune global plan false, %s", ex.what());
            return false;
        }
        return true;
    }

    //提取待优化路径，current-goal-idx是待优化路径的终点在全局路径中的索引
    bool TebLocalPlanner::TransformGlobalPlan(int *current_goal_idx) {

        transformed_plan_.clear();

        try {
            if (global_plan_.poses.empty()) {
                ROS_ERROR("Received plan with zero length");
                *current_goal_idx = 0;
                return false;
            }
            UpdateGlobalToPlanTranform();

            //机器人在map坐标系位姿Pmap
            tf::Stamped<tf::Pose> robot_pose;
            tf_.lock()->transformPose(global_plan_.poses.front().header.frame_id, robot_tf_pose_, robot_pose);

            double dist_threshold = std::max(costmap_->GetSizeXCell() * costmap_->GetResolution() / 2.0,
                                             costmap_->GetSizeYCell() * costmap_->GetResolution() / 2.0);
            dist_threshold *= 0.85;

            int i = 0;
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist = 1e10;
            double new_sq_dist = 0;
            //这个循环的含义：以一维举个例子
            //全局路径p： 1  2  3  4  5  6  7; 机器人位姿  1.8
            //则以下循环结束时，i=2，p[i]=3
            //再下一个循环中，会把 3 4 5 加入到待优化路径，这三个坐标组成的路径长度为2m（注意这三个坐标都要转到odom系下）
            //在这个函数外，会将机器人位姿也加入待优化路径，待优化路径为： 1.8  3  4  5 （注意这几个坐标都要转到odom系下）
            while (i < (int) global_plan_.poses.size()) {
                double x_diff = robot_pose.getOrigin().x() - global_plan_.poses[i].pose.position.x;
                double y_diff = robot_pose.getOrigin().y() - global_plan_.poses[i].pose.position.y;
                new_sq_dist = x_diff * x_diff + y_diff * y_diff;
                if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) {
                    sq_dist = new_sq_dist;
                    break;
                }
                sq_dist = new_sq_dist;
                ++i;
            }

            tf::Stamped<tf::Pose> tf_pose;
            geometry_msgs::PoseStamped newer_pose;

            double plan_length = 0;

            //(cxn)只有 mapPath 中的 waypoints组成的路径长度在cut-lookahead-dist-(默认配置2m)以内，就把waypoint转到odom系下
            while (i < (int) global_plan_.poses.size() &&
                   sq_dist <= sq_dist_threshold && (cut_lookahead_dist_ <= 0 || plan_length <= cut_lookahead_dist_)) {
                const geometry_msgs::PoseStamped &pose = global_plan_.poses[i];
                tf::poseStampedMsgToTF(pose, tf_pose);
                tf_pose.setData(plan_to_global_transform_ * tf_pose);
                tf_pose.stamp_ = plan_to_global_transform_.stamp_;
                tf_pose.frame_id_ = global_frame_;
                tf::poseStampedTFToMsg(tf_pose, newer_pose);
                auto temp = DataConverter::LocalConvertGData(newer_pose.pose);
                DataBase data_pose(temp.first, temp.second);

                transformed_plan_.push_back(data_pose);

                double x_diff = robot_pose.getOrigin().x() - global_plan_.poses[i].pose.position.x;
                double y_diff = robot_pose.getOrigin().y() - global_plan_.poses[i].pose.position.y;
                sq_dist = x_diff * x_diff + y_diff * y_diff;

                if (i > 0 && cut_lookahead_dist_ > 0) {
                    plan_length += Distance(global_plan_.poses[i - 1].pose.position.x,
                                            global_plan_.poses[i - 1].pose.position.y,
                                            global_plan_.poses[i].pose.position.x,
                                            global_plan_.poses[i].pose.position.y);
                }

                ++i;
            }

            if (transformed_plan_.empty()) {
                tf::poseStampedMsgToTF(global_plan_.poses.back(), tf_pose);
                tf_pose.setData(plan_to_global_transform_ * tf_pose);
                tf_pose.stamp_ = plan_to_global_transform_.stamp_;
                tf_pose.frame_id_ = global_frame_;
                tf::poseStampedTFToMsg(tf_pose, newer_pose);

                auto temp = DataConverter::LocalConvertGData(newer_pose.pose);
                DataBase data_pose(temp.first, temp.second);

                transformed_plan_.push_back(data_pose);

                if (current_goal_idx) {
                    *current_goal_idx = int(global_plan_.poses.size()) - 1;
                }
            } else {
                if (current_goal_idx) {
                    *current_goal_idx = i - 1;
                }
            }
        }
        catch (tf::LookupException &ex) {
            //LOG_ERROR << "transform error, " << ex.what();
            return false;
        }
        catch (tf::ConnectivityException &ex) {
            ROS_ERROR("Connectivity Error: %s", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException &ex) {
            ROS_ERROR("Extrapolation Error: %s", ex.what());
            if (global_plan_.poses.size() > 0) {
                ROS_INFO("Global Frame: %s Plan Frame size %d : %s", global_frame_.c_str(),
                         (unsigned int) global_plan_.poses.size(),
                         global_plan_.poses[0].header.frame_id.c_str());
            }
            return false;
        }

        return true;
    }

    //(cxn)每次TEB时，并不会对整条mapPath做优化，而是从mapPath中截取前一段（默认配置是mapPath的前2m），截完之后，要估计一下末尾waypoint的yaw
    //如 mapPath： s1，s2，...，sn
    //截取 sj，...,st，估计st的yaw
    //怎么算呢，大致方法是：取st 后一定数量的 si，如 st,st+1,...st+3，用atan2算这几段内点theta： theta1，...theta3，其平均值作为 st的yaw
    double TebLocalPlanner::EstimateLocalGoalOrientation(const DataBase &local_goal,
                                                         int current_goal_idx, int moving_average_length) const {
        int n = (int) global_plan_.poses.size();

        if (current_goal_idx > n - moving_average_length - 2) {
            if (current_goal_idx >= n - 1) {
                return local_goal.GetTheta();
            } else {
                tf::Quaternion global_orientation;
                tf::quaternionMsgToTF(global_plan_.poses.back().pose.orientation, global_orientation);
                return tf::getYaw(plan_to_global_transform_.getRotation() * global_orientation);
            }
        }

        moving_average_length = std::min(moving_average_length, n - current_goal_idx - 1);

        std::vector<double> candidates;
        tf::Stamped<tf::Pose> tf_pose_k;
        tf::Stamped<tf::Pose> tf_pose_kp1;

        const geometry_msgs::PoseStamped &pose_k = global_plan_.poses.at(current_goal_idx);
        tf::poseStampedMsgToTF(pose_k, tf_pose_k);
        tf_pose_kp1.setData(plan_to_global_transform_ * tf_pose_k);

        int range_end = current_goal_idx + moving_average_length;
        for (int i = current_goal_idx; i < range_end; ++i) {

            const geometry_msgs::PoseStamped &pose_k1 = global_plan_.poses.at(i + 1);
            tf::poseStampedMsgToTF(pose_k1, tf_pose_kp1);
            tf_pose_kp1.setData(plan_to_global_transform_ * tf_pose_kp1);

            candidates.push_back(std::atan2(tf_pose_kp1.getOrigin().getY() - tf_pose_k.getOrigin().getY(),
                                            tf_pose_kp1.getOrigin().getX() - tf_pose_k.getOrigin().getX()));

            if (i < range_end - 1)
                tf_pose_k = tf_pose_kp1;
        }
        return AverageAngles(candidates);
    }

    void TebLocalPlanner::UpdateObstacleWithCostmap(Eigen::Vector2d local_goal) {

        //Eigen::Vector2d robot_orient = robot_pose_.OrientationUnitVec();
        Eigen::Vector2d goal_orient = local_goal - robot_pose_.GetPosition();

        for (unsigned int i = 0; i < costmap_->GetSizeXCell() - 1; ++i) {
            for (unsigned int j = 0; j < costmap_->GetSizeYCell() - 1; ++j) {
                if (costmap_->GetCost(i, j) == or_local_planner::LETHAL_OBSTACLE) {
                    Eigen::Vector2d obs;
                    costmap_->Map2World(i, j, obs.coeffRef(0), obs.coeffRef(1));

                    Eigen::Vector2d obs_dir = obs - robot_pose_.GetPosition();
                    if (obs_dir.dot(goal_orient) < 0 && obs_dir.norm() > osbtacle_behind_robot_dist_) {
                        continue;
                    }

                    obst_vector_.push_back(ObstaclePtr(new PointObstacle(obs)));
                }
            } // for y cell size
        }     // for x cell size
    }

    //用设定值global-plan-viapoint-sep分割待优化路径，得到一些必须通过的轨迹点
    void TebLocalPlanner::UpdateViaPointsContainer() {

        via_points_.clear();

        double min_separation = param_config_.trajectory_opt().global_plan_viapoint_sep();
        if (min_separation < 0) {
            return;
        }

        std::size_t prev_idx = 0;
        for (std::size_t i = 1; i <
                                transformed_plan_.size(); ++i) { // skip first one, since we do not need any point before the first min_separation [m]

            if (Distance(transformed_plan_[prev_idx].GetPosition().coeff(0),
                         transformed_plan_[prev_idx].GetPosition().coeff(1),
                         transformed_plan_[i].GetPosition().coeff(0), transformed_plan_[i].GetPosition().coeff(1)) <
                min_separation) {
                continue;
            }

            auto temp = transformed_plan_[i].GetPosition();
            via_points_.push_back(temp);
            prev_idx = i;
        }
    }

    void TebLocalPlanner::UpdateRobotPose() {
        //获取机器人在odom系位姿：Podom
        local_cost_.lock()->GetRobotPose(robot_tf_pose_);
        Eigen::Vector2d position;
        position.coeffRef(0) = robot_tf_pose_.getOrigin().getX();
        position.coeffRef(1) = robot_tf_pose_.getOrigin().getY();
        robot_pose_ = DataBase(position, tf::getYaw(robot_tf_pose_.getRotation()));
    }

    void TebLocalPlanner::UpdateRobotVel() {
        // tf::Stamped<tf::Pose> robot_vel_tf;
        // odom_info_.GetVel(robot_vel_tf);
        // robot_current_vel_.linear.x = robot_vel_tf.getOrigin().getX();
        // robot_current_vel_.linear.y = robot_vel_tf.getOrigin().getY();
        // robot_current_vel_.angular.z = tf::getYaw(robot_vel_tf.getRotation());
    }

    //(cxn)获取全局路径起点的 Todom-map
    void TebLocalPlanner::UpdateGlobalToPlanTranform() {
        tf_.lock()->waitForTransform(global_frame_, ros::Time::now(),
                                     global_plan_.poses.front().header.frame_id,
                                     global_plan_.poses.front().header.stamp,
                                     global_plan_.poses.front().header.frame_id, ros::Duration(0.5));
        tf_.lock()->lookupTransform(global_frame_, ros::Time(),
                                    global_plan_.poses.front().header.frame_id, global_plan_.poses.front().header.stamp,
                                    global_plan_.poses.front().header.frame_id, plan_to_global_transform_);
    }

    void TebLocalPlanner::SaturateVelocity(double &vx, double &vy, double &omega, double max_vel_x, double max_vel_y,
                                           double max_vel_theta, double max_vel_x_backwards) const {

        if (vx > max_vel_x) {
            vx = max_vel_x;
        }

        if (max_vel_x_backwards <= 0) {
            ROS_INFO("Do not choose max_vel_x_backwards to be <=0. "
                     "Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
        } else if (vx < -max_vel_x_backwards) {
            vx = -max_vel_x_backwards;
        }

        if (vy > max_vel_y) {
            vy = max_vel_y;
        } else if (vy < -max_vel_y) {
            vy = -max_vel_y;
        }

        if (omega > max_vel_theta) {
            omega = max_vel_theta;
        } else if (omega < -max_vel_theta) {
            omega = -max_vel_theta;
        }
    }

    double TebLocalPlanner::ConvertTransRotVelToSteeringAngle(double v, double omega, double wheelbase,
                                                              double min_turning_radius) const {
        if (omega == 0 || v == 0) {
            return 0;
        }

        double radius = v / omega;

        if (fabs(radius) < min_turning_radius) {
            radius = double(g2o::sign(radius)) * min_turning_radius;
        }

        return std::atan(wheelbase / radius);
    }

    or_common::ErrorInfo TebLocalPlanner::Initialize(std::shared_ptr<or_costmap::CostmapInterface> local_cost,
                                                     std::shared_ptr<tf::TransformListener> tf,
                                                     LocalVisualizationPtr visual) {

        if (!is_initialized_) {
            oscillation_ = std::chrono::system_clock::now();
            tf_ = tf;
            local_cost_ = local_cost;

            std::string full_path = ros::package::getPath("or_planning") +
                                    "/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt";
            or_common::ReadProtoFromTextFile(full_path.c_str(), &param_config_);
            if (&param_config_ == nullptr) {
                ROS_ERROR("error occur when loading config file");
                or_common::ErrorInfo read_file_error(or_common::ErrorCode::LP_ALGORITHM_INITILIZATION_ERROR,
                                                     "load algorithm param file failed");
                is_initialized_ = false;
                ROS_ERROR("%s", read_file_error.error_msg().c_str());
                return read_file_error;
            }

            max_vel_x_ = param_config_.kinematics_opt().max_vel_x();
            max_vel_y_ = param_config_.kinematics_opt().max_vel_y();
            max_vel_theta_ = param_config_.kinematics_opt().max_vel_theta();
            max_vel_x_backwards = param_config_.kinematics_opt().max_vel_x_backwards();
            free_goal_vel_ = param_config_.tolerance_opt().free_goal_vel();

            xy_goal_tolerance_ = param_config_.tolerance_opt().xy_goal_tolerance();
            yaw_goal_tolerance_ = param_config_.tolerance_opt().yaw_goal_tolerance();

            cut_lookahead_dist_ = param_config_.trajectory_opt().max_global_plan_lookahead_dist();
            fesiable_step_look_ahead_ = param_config_.trajectory_opt().feasibility_check_no_poses();

            osbtacle_behind_robot_dist_ = param_config_.obstacles_opt().costmap_obstacles_behind_robot_dist();

            global_plan_overwrite_orientation_ = param_config_.trajectory_opt().global_plan_overwrite_orientation();

            global_frame_ = local_cost_.lock()->GetGlobalFrameID();

            visual_ = visual;

            costmap_ = local_cost_.lock()->GetLayeredCostmap()->GetCostMap();

            robot_cost_ = std::make_shared<or_local_planner::RobotPositionCost>(*costmap_);

            obst_vector_.reserve(200);

            //(cxn)定义机器人类型：点
            RobotFootprintModelPtr robot_model = GetRobotFootprintModel(param_config_);
            optimal_ = TebOptimalPtr(new TebOptimal(param_config_, &obst_vector_, robot_model, visual_, &via_points_));

            std::vector<geometry_msgs::Point> robot_footprint;
            robot_footprint = local_cost_.lock()->GetRobotFootprint();
            //robot_footprint_ = DataConverter::LocalConvertGData(robot_footprint);

            or_costmap::RobotPose robot_pose;
            local_cost_.lock()->GetRobotPose(robot_pose);
            auto temp_pose = DataConverter::LocalConvertRMData(robot_pose);
            robot_pose_ = DataBase(temp_pose.first, temp_pose.second);
            robot_footprint_.push_back(robot_pose_.GetPosition());
            last_robot_pose_ = robot_pose_;

            RobotPositionCost::CalculateMinAndMaxDistances(robot_footprint_,
                                                           robot_inscribed_radius_,
                                                           robot_circumscribed_radius);
            // odom_info_.SetTopic(param_config_.opt_frame().odom_frame());

            is_initialized_ = true;
            ROS_INFO("local algorithm initialize ok");
            return or_common::ErrorInfo(or_common::ErrorCode::OK);
        }

        return or_common::ErrorInfo(or_common::ErrorCode::OK);
    }

    void TebLocalPlanner::RegisterErrorCallBack(ErrorInfoCallback error_callback) {
        error_callback_ = error_callback;
    }

    bool TebLocalPlanner::CutAndTransformGlobalPlan(int *current_goal_idx) {
        if (!transformed_plan_.empty()) {
            transformed_plan_.clear();
        }
    }

    bool TebLocalPlanner::SetPlanOrientation() {
        if (global_plan_.poses.size() < 2) {
            ROS_WARN("can not compute the orientation because the global plan size is: %d",
                     (int) global_plan_.poses.size());
            return false;
        } else {
            //auto goal = DataConverter::LocalConvertGData(global_plan_.poses.back().pose);
            //auto line_vector = (robot_pose_.GetPosition() - goal.first);
            //auto  orientation = GetOrientation(line_vector);
            for (int i = 0; i < global_plan_.poses.size() - 1; ++i) {
                auto pose = DataConverter::LocalConvertGData(global_plan_.poses[i].pose);
                auto next_pose = DataConverter::LocalConvertGData(global_plan_.poses[i + 1].pose);
                double x = global_plan_.poses[i + 1].pose.position.x - global_plan_.poses[i].pose.position.x;
                double y = global_plan_.poses[i + 1].pose.position.y - global_plan_.poses[i].pose.position.y;
                double angle = atan2(y, x);
                auto quaternion = EulerToQuaternion(0, 0, angle);
                global_plan_.poses[i].pose.orientation.w = quaternion[0];
                global_plan_.poses[i].pose.orientation.x = quaternion[1];
                global_plan_.poses[i].pose.orientation.y = quaternion[2];
                global_plan_.poses[i].pose.orientation.z = quaternion[3];
            }
        }
    }
} // namespace or_local_planner