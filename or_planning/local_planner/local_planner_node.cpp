//
// Created by cxn on 2020/7/5.
//

/*
 * Loop是规划线程：（用于规划，并对node_state_赋值）
 *   local_planner_->Initialize()
 *   while(node_state_==running):
 *      规划local_planner_->ComputeVelocityCommands();
 *      若规划连续5次失败 node_state_=failure
 *      若机器人已到目标位置时，则node_state_=success
 *      线程靠plan_condition_的wait_for延时，默认是25ms一次循环
 *   vel_pub_发送停止指令
 *
 * GoalCallback是action执行线程(开启、终止、唤醒Loop)(另外，其只会在终止Loop线程时，才会修改node_state_)
 *   给local_planner_设置目标
 *      若此时node_state_为IDLE，说明Loop是空的，则新建Loop（刚进入Loop时，默认wait_for延时0微秒）
 *      若此时node_state_为running，依靠plan_condition_唤醒Loop(也就是说若Loop在靠wait_for延时，直接就跳出延时)
 *   while(ros::ok()):
 *      若PlanThread中的规划算法有异常,则发布反馈
 *      若node_state_为success or failure
 *              终止Loop线程运行（将node_state_修改为IDLE）
 */


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
            local_planner_ = std::make_unique<TebLocalPlanner>();
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

        //触发plan线程
        {
            std::unique_lock<std::mutex> plan_lock(plan_mutex_);
            local_planner_->SetPlan(command->route, local_goal_);
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
                sleep_time = std::chrono::milliseconds(0);
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