#ifndef OR_EXECUTOR_CHASSIS_EXECUTOR_H
#define OR_EXECUTOR_CHASSIS_EXECUTOR_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include "or_msgs/GlobalPlannerAction.h"
#include "or_msgs/LocalPlannerAction.h"
#include "or_msgs/TwistAccel.h"
#include "geometry_msgs/Twist.h"


namespace or_executor {

    /**
     * @brief Behavior state
     */
    enum class ExcutionState {
        RUNNING,   ///< Running state in process
        SUCCESS,   ///< Success state as result
        FAILURE,   ///< Failure state as result
        IDLE,      ///< Idle state, state as default or after cancellation
    };


    /***
     * @brief Chassis Executor to execute different abstracted task for chassis module
     */
    class ChassisExecutor {
    public:

        /**
         * @brief Chassis execution mode for different tasks
         */
        enum class ExcutionMode {
            IDLE_MODE,            ///< Default idle mode with no task
            GOAL_MODE,            ///< Goal-targeted task mode using global and local planner
        };

        /**
         * @brief Constructor of ChassisExecutor
         */
        ChassisExecutor();

        ~ChassisExecutor() = default;

        /**
         * @brief Execute the goal-targeted task using global and local planner with actionlib
         * @param goal Given taget goal
         */
        void Execute(const geometry_msgs::PoseStamped &goal);

        /**
         * @brief Update the current chassis executor state
         * @return Current chassis executor state(same with behavior state)
         */
        ExcutionState Update();

        /**
         * @brief Cancel the current task and deal with the mode transition
         */
        void Cancel();

    private:
        /***
         * @brief Global planner actionlib feedback callback function to send the global planner path to local planner
         * @param global_planner_feedback  Global planner actionlib feedback, which mainly consists of global planner path output
         */
        void GlobalPlannerFeedbackCallback(const or_msgs::GlobalPlannerFeedbackConstPtr &global_planner_feedback);

        //! execution mode of the executor
        ExcutionMode execution_mode_;
        //! execution state of the executor (same with behavior state)
        ExcutionState execution_state_;

        //! global planner actionlib client
        actionlib::SimpleActionClient<or_msgs::GlobalPlannerAction> global_planner_client_;
        //! local planner actionlib client
        actionlib::SimpleActionClient<or_msgs::LocalPlannerAction> local_planner_client_;
        //! global planner actionlib goal
        or_msgs::GlobalPlannerGoal global_planner_goal_;
        //! local planner actionlib goal
        or_msgs::LocalPlannerGoal local_planner_goal_;

        //! velocity control publisher in ROS
        ros::Publisher cmd_vel_pub_;
        //! zero twist in form of ROS geometry_msgs::Twist
        geometry_msgs::Twist zero_twist_;
    };
}


#endif //OR_EXECUTOR_CHASSIS_EXECUTOR_H