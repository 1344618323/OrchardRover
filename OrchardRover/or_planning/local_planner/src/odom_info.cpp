//
// Created by cxn on 2020/7/8.
//

#include "odom_info.h"

namespace or_local_planner {
    OdomInfo::OdomInfo(std::string topic) {
        SetTopic(topic);
    }

    void OdomInfo::OdomCB(const nav_msgs::Odometry::ConstPtr &msg) {
        std::unique_lock<std::mutex> lock(mutex_);
        odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
        odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
        odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
        odom_.child_frame_id = msg->child_frame_id;
    }

    void OdomInfo::GetVel(tf::Stamped<tf::Pose> &vel) {
        geometry_msgs::Twist temp_vel;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            temp_vel.linear.x = odom_.twist.twist.linear.x;
            temp_vel.linear.y = odom_.twist.twist.linear.y;
            temp_vel.angular.z = odom_.twist.twist.angular.z;

            vel.frame_id_ = odom_.child_frame_id;
        }
        vel.setData(tf::Transform(tf::createQuaternionFromYaw(temp_vel.angular.z),
                                  tf::Vector3(temp_vel.linear.x, temp_vel.linear.y, 0)));
        vel.stamp_ = ros::Time();
    }

    void OdomInfo::SetTopic(std::string topic) {
        if (topic_ == topic) {
            return;
        } else {
            topic_ = topic;
            if (topic == "") {
                sub_.shutdown();
                return;
            } else {
                ros::NodeHandle nh;
                sub_ = nh.subscribe<nav_msgs::Odometry>(topic_, 1, boost::bind(&OdomInfo::OdomCB, this, _1));
            }
        }
    }
}