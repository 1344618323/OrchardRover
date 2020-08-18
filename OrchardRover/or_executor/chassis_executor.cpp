#include "chassis_executor.h"

namespace or_executor {
    ChassisExecutor::ChassisExecutor() : execution_mode_(ExcutionMode::IDLE_MODE),
                                         execution_state_(ExcutionState::IDLE),
                                         global_planner_client_("global_planner_node_action", true),
                                         local_planner_client_("local_planner_node_action", true) {
        ros::NodeHandle nh;
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        global_planner_client_.waitForServer();
        ROS_INFO("Global planer server start!");
        local_planner_client_.waitForServer();
        ROS_INFO("Local planer server start!");
    }

    void ChassisExecutor::Execute(const geometry_msgs::PoseStamped &goal) {
        execution_mode_ = ExcutionMode::GOAL_MODE;
        global_planner_goal_.goal = goal;
        global_planner_client_.sendGoal(global_planner_goal_,
                                        actionlib::SimpleActionClient<or_msgs::GlobalPlannerAction>::SimpleDoneCallback(),
                                        actionlib::SimpleActionClient<or_msgs::GlobalPlannerAction>::SimpleActiveCallback(),
                                        boost::bind(&ChassisExecutor::GlobalPlannerFeedbackCallback, this, _1));
    }

    ExcutionState ChassisExecutor::Update() {
        actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::LOST;
        switch (execution_mode_) {
            case ExcutionMode::IDLE_MODE:
                execution_state_ = ExcutionState::IDLE;
                break;

            case ExcutionMode::GOAL_MODE:
                state = global_planner_client_.getState();
                if (state == actionlib::SimpleClientGoalState::ACTIVE) {
                    ROS_INFO("%s : ACTIVE", __FUNCTION__);
                    execution_state_ = ExcutionState::RUNNING;
                } else if (state == actionlib::SimpleClientGoalState::PENDING) {
                    ROS_INFO("%s : PENDING", __FUNCTION__);
                    execution_state_ = ExcutionState::RUNNING;
                } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("%s : SUCCEEDED", __FUNCTION__);
                    execution_state_ = ExcutionState::SUCCESS;
                } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                    ROS_INFO("%s : ABORTED", __FUNCTION__);
                    execution_state_ = ExcutionState::FAILURE;
                } else {
                    ROS_ERROR("Error: %s", state.toString().c_str());
                    execution_state_ = ExcutionState::FAILURE;
                }
                break;

            default:
                ROS_ERROR("Wrong Execution Mode");
        }
        return execution_state_;
    };

    void ChassisExecutor::Cancel() {
        switch (execution_mode_) {
            case ExcutionMode::IDLE_MODE:
                ROS_WARN("Nothing to be canceled.");
                break;

            case ExcutionMode::GOAL_MODE:
                global_planner_client_.cancelGoal();
                local_planner_client_.cancelGoal();
                execution_mode_ = ExcutionMode::IDLE_MODE;
                break;

            default:
                ROS_ERROR("Wrong Execution Mode");
        }
    }

    void ChassisExecutor::GlobalPlannerFeedbackCallback(
            const or_msgs::GlobalPlannerFeedbackConstPtr &global_planner_feedback) {
        if (!global_planner_feedback->path.poses.empty()) {
            local_planner_goal_.route = global_planner_feedback->path;
            local_planner_client_.sendGoal(local_planner_goal_);
        }
    }

} // namespace or_executor