//
// Created by cxn on 2020/7/5.
//
#include "local_planner_node.h"

namespace or_local_planner {
    using or_common::ErrorCode;
    using or_common::ErrorInfo;
    using or_common::NodeState;

    LocalPlannerNode::LocalPlannerNode() :
            local_planner_nh_("~"),
            as_(local_planner_nh_, "/local_planner_node_action", boost::bind(&LocalPlannerNode::ExcuteCB, this, _1),
                false),
            initialized_(false), node_state_(or_common::NodeState::IDLE),
            node_error_info_(or_common::ErrorCode::OK), max_error_(5),
            local_cost_(nullptr), tf_(nullptr) {
        if (Init().IsOK()) {
            ROS_INFO("local planner initialize completed.");
            as_.start();
        } else {
            ROS_WARN("local planner initialize failed.");
            SetNodeState(NodeState::FAILURE);
        }
    }

    LocalPlannerNode::~LocalPlannerNode() {
        StopPlanning();
    }

    or_common::ErrorInfo LocalPlannerNode::Init() {
        ROS_INFO("local planner start");
        LocalAlgorithms local_algorithms;
        std::string full_path = ros::package::getPath("or_planning") + "/local_planner/config/local_planner.prototxt";
        or_common::ReadProtoFromTextFile(full_path.c_str(), &local_algorithms);
        if (&local_algorithms == nullptr) {
            return or_common::ErrorInfo(or_common::ErrorCode::LP_INITILIZATION_ERROR,
                                        "Cannot load local planner protobuf configuration file.");
        }
        selected_algorithm_ = local_algorithms.selected_algorithm();
        frequency_ = local_algorithms.frequency();
        tf_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

        std::string map_path =
                ros::package::getPath("or_costmap") + "/config/costmap_parameter_config_for_local_plan.prototxt";
        local_cost_ = std::make_shared<or_costmap::CostmapInterface>("local_costmap",
                                                                     *tf_,
                                                                     map_path.c_str());
        if (selected_algorithm_ == "timed_elastic_band") {
            local_planner_;
        }
        if (local_planner_ == nullptr) {
            ROS_ERROR("local planner algorithm instance can't be loaded");
            return or_common::ErrorInfo(or_common::ErrorCode::LP_INITILIZATION_ERROR,
                                        "local planner algorithm instance can't be loaded");
        }

        std::string name;
        visual_frame_ = local_cost_->GetGlobalFrameID();
        visual_ = std::make_shared<LocalVisualization>(local_planner_nh_, visual_frame_);
        vel_pub_ = local_planner_nh_.advertise<or_msgs::TwistAccel>("/cmd_vel_acc", 5);

        return or_common::ErrorInfo(or_common::ErrorCode::OK);
    }

    void LocalPlannerNode::ExcuteCB(const or_msgs::LocalPlannerGoal::ConstPtr &command) {
        ErrorInfo error_info = GetErrorInfo();
        NodeState node_state = GetNodeState();

        if (plan_mtx_.try_lock()) {
            local_planner_->SetPlan(command->route, local_goal_);
            plan_mtx_.unlock();
            plan_condition_.notify_one();
        }

        ROS_INFO("Send Plan!");
        if (node_state == NodeState::IDLE) {
            StartPlanning();
        }

        while (ros::ok()) {
            if (as_.isPreemptRequested()) {
                ROS_INFO("Action Preempted");
                if (as_.isNewGoalAvailable()) {
                    as_.setPreempted();
                    break;
                } else {
                    as_.setPreempted();
                    StopPlanning();
                    break;
                }
            }
            node_state = GetNodeState();
            error_info = GetErrorInfo();

            if (node_state == NodeState::RUNNING || node_state == NodeState::SUCCESS
                || node_state == NodeState::FAILURE) {
                or_msgs::LocalPlannerFeedback feedback;
                or_msgs::LocalPlannerResult result;
                if (!error_info.IsOK()) {
                    feedback.error_code = error_info.error_code();
                    feedback.error_msg = error_info.error_msg();
                    SetErrorInfo(or_common::ErrorInfo::OK());

                    as_.publishFeedback(feedback);
                }
                if (node_state == NodeState::SUCCESS) {
                    result.error_code = error_info.error_code();
                    as_.setSucceeded(result, error_info.error_msg());
                    StopPlanning();
                    break;
                } else if (node_state == NodeState::FAILURE) {
                    result.error_code = error_info.error_code();
                    as_.setAborted(result, error_info.error_msg());
                    StopPlanning();
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void LocalPlannerNode::Loop() {

        or_common::ErrorInfo error_info = local_planner_->Initialize(local_cost_, tf_, visual_);
        if (error_info.IsOK()) {
            ROS_INFO("local planner algorithm initialize completed.");
        } else {
            ROS_WARN("local planner algorithm initialize failed.");
            SetNodeState(NodeState::FAILURE);
            SetErrorInfo(error_info);
        }
        std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
        int error_count = 0;

        while (GetNodeState() == NodeState::RUNNING) {
            std::unique_lock<std::mutex> plan_lock(plan_mutex_);
            plan_condition_.wait_for(plan_lock, sleep_time);
            auto begin = std::chrono::steady_clock::now();

            or_common::ErrorInfo error_info = local_planner_->ComputeVelocityCommands(cmd_vel_);

            auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - begin);
            int need_time = 1000 / frequency_;
            sleep_time = std::chrono::milliseconds(need_time) - cost_time;

            if (sleep_time <= std::chrono::milliseconds(0)) {
                //LOG_WARNING << "The time planning once is " << cost_time.count() << " beyond the expected time "
                //        << std::chrono::milliseconds(50).count();
                sleep_time = std::chrono::milliseconds(0);
                //SetErrorInfo(ErrorInfo(ErrorCode::GP_TIME_OUT_ERROR, "Planning once time out."));
            }

            if (error_info.IsOK()) {
                error_count = 0;
                vel_pub_.publish(cmd_vel_);
                if (local_planner_->IsGoalReached()) {
                    SetNodeState(NodeState::SUCCESS);
                }
            } else if (error_count > max_error_ && max_error_ > 0) {
                ROS_WARN("Can not finish plan with max retries( %d  )", max_error_);
                error_info = or_common::ErrorInfo(or_common::ErrorCode::LP_MAX_ERROR_FAILURE, "over max error.");
                SetNodeState(NodeState::FAILURE);
            } else {
                error_count++;
                ROS_ERROR("Can not get cmd_vel for once. %s error count:  %d", error_info.error_msg().c_str(),
                          error_count);
            }

            SetErrorInfo(error_info);
        }

        cmd_vel_.twist.linear.x = 0;
        cmd_vel_.twist.linear.y = 0;
        cmd_vel_.twist.angular.z = 0;
        cmd_vel_.accel.linear.x = 0;
        cmd_vel_.accel.linear.y = 0;
        cmd_vel_.accel.angular.z = 0;
        vel_pub_.publish(cmd_vel_);
    }

    void LocalPlannerNode::SetErrorInfo(const or_common::ErrorInfo error_info) {
        std::lock_guard<std::mutex> guard(node_error_info_mtx_);
        node_error_info_ = error_info;
    }

    void LocalPlannerNode::SetNodeState(const or_common::NodeState &node_state) {
        std::lock_guard<std::mutex> guard(node_state_mtx_);
        node_state_ = node_state;
    }

    or_common::NodeState LocalPlannerNode::GetNodeState() {
        std::lock_guard<std::mutex> guard(node_state_mtx_);
        return node_state_;
    }

    or_common::ErrorInfo LocalPlannerNode::GetErrorInfo() {
        std::lock_guard<std::mutex> guard(node_error_info_mtx_);
        return node_error_info_;
    }

    void LocalPlannerNode::StartPlanning() {
        if (local_planner_thread_.joinable()) {
            local_planner_thread_.join();
        }

        SetNodeState(or_common::NodeState::RUNNING);
        local_planner_thread_ = std::thread(std::bind(&LocalPlannerNode::Loop, this));
    }

    void LocalPlannerNode::StopPlanning() {
        SetNodeState(or_common::IDLE);
        if (local_planner_thread_.joinable()) {
            local_planner_thread_.join();
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner_node", ros::init_options::NoSigintHandler);
    or_local_planner::LocalPlannerNode local_planner;
    ros::spin();
    local_planner.StopPlanning();
    return 0;
}