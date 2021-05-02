#include "local_visualization.h"

namespace or_local_planner {
    LocalVisualization::LocalVisualization() : initialized_(false) {

    }

    LocalVisualization::LocalVisualization(ros::NodeHandle &nh, const std::string &visualize_frame) : initialized_(
            false) {
        Initialization(nh, visualize_frame);
    }

    void LocalVisualization::Initialization(ros::NodeHandle &nh, const std::string &visualize_frame) {
        if (initialized_) {

        }

        visual_frame_ = visualize_frame;
        local_planner_ = nh.advertise<nav_msgs::Path>("trajectory", 1);

        initialized_ = true;
    }

    void LocalVisualization::PublishLocalPlan(const TebVertexConsole &vertex_console) const {

        if (local_planner_.getNumSubscribers() > 0) {
            nav_msgs::Path local_plan;
            local_plan.header.frame_id = visual_frame_;
            local_plan.header.stamp = ros::Time::now();

            for (int i = 0; i < vertex_console.SizePoses(); ++i) {
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.frame_id = local_plan.header.frame_id;
                pose_stamped.header.stamp = local_plan.header.stamp;
                pose_stamped.pose.position.x = vertex_console.Pose(i).GetPosition().coeffRef(0);
                pose_stamped.pose.position.y = vertex_console.Pose(i).GetPosition().coeffRef(1);
                pose_stamped.pose.position.z = 0;
                pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(vertex_console.Pose(i).GetTheta());
                local_plan.poses.push_back(pose_stamped);
            }
            local_planner_.publish(local_plan);
        }
    }

} // namespace or_local_planner

