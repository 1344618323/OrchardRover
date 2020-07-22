#include <ros/ros.h>
#include "chassis_executor.h"
#include "goal_behavior.h"
#include <mutex>

bool new_goal_ = false;
geometry_msgs::PoseStamped goal_;
std::mutex new_goal_mtx_;

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal) {
    std::lock_guard<std::mutex> guard(new_goal_mtx_);
    new_goal_ = true;
    goal_ = *goal;
}

inline bool IsNewGoal() {
    std::lock_guard<std::mutex> guard(new_goal_mtx_);
    if (new_goal_) {
        new_goal_ = false;
        return true;
    } else {
        return false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "or_executor_node");

    ros::NodeHandle rviz_nh("/move_base_simple");
    ros::Subscriber rviz_sub = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &GoalCallback);

    std::unique_ptr<or_executor::ChassisExecutor> chassis_executor = std::make_unique<or_executor::ChassisExecutor>();

    or_executor::GoalBehavior goal_behavior(chassis_executor.get());

    ros::Rate rate(5);

    while (ros::ok()) {
        ros::spinOnce();
        if (IsNewGoal())
            goal_behavior.Run(goal_);
        rate.sleep();
    }

    return 0;
}