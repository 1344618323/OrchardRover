#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "fastslam_test.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle n;
    ros::Publisher particlepose_pub_ = n.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
    geometry_msgs::PoseArray particlecloud_msg_;
    particlecloud_msg_.header.frame_id = "map";

    ros::Publisher marker_pub_ = n.advertise<visualization_msgs::Marker>("lmcloud_msg_", 2);
    visualization_msgs::Marker lmekfcloud;
    lmekfcloud.header.frame_id = "map";
    lmekfcloud.action = visualization_msgs::Marker::ADD;
    lmekfcloud.type = visualization_msgs::Marker::POINTS;
    lmekfcloud.pose.position.x = 0;
    lmekfcloud.pose.position.y = 0;
    lmekfcloud.pose.position.z = 0;
    lmekfcloud.pose.orientation.x = 0.0;
    lmekfcloud.pose.orientation.y = 0.0;
    lmekfcloud.pose.orientation.z = 0.0;
    lmekfcloud.pose.orientation.w = 1.0;
    lmekfcloud.scale.x = 0.05;
    lmekfcloud.scale.y = 0.05;
    lmekfcloud.color.r = 0.0;
    lmekfcloud.color.g = 0.0;
    lmekfcloud.color.b = 1.0;
    lmekfcloud.color.a = 1.0;//一定要初始化，否则默认为0，看不见！

    lmekfcloud.ns = "lm";
    lmekfcloud.id = 0;


    tf::TransformBroadcaster br;

    geometry_msgs::TransformStamped gimbal_tf_;
    gimbal_tf_.header.frame_id = "map";
    gimbal_tf_.child_frame_id = "odom";

    ros::Rate loop_rate(10);
    int cnt = 0;
    while (ros::ok()) {
        if (cnt++ == 10) {
            if (particlepose_pub_.getNumSubscribers() > 0) {
                // Publish the resulting particle cloud
                particlecloud_msg_.poses.resize(100);
                for (int i = 0; i < 100; i++) {
                    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(drand48() * 2 * M_PI - M_PI),
                                             tf::Vector3(drand48() * 5, drand48() * 5, 0)),
                                    particlecloud_msg_.poses[i]);
                }
                particlecloud_msg_.header.stamp = ros::Time::now();
                particlepose_pub_.publish(particlecloud_msg_);
            }
            if (marker_pub_.getNumSubscribers() > 0) {
                // Publish the resulting particle cloud
                lmekfcloud.points.resize(1000);
                for (int i = 0; i < 1000; i++) {
                    lmekfcloud.points[i].x = drand48() * 5 + 7;
                    lmekfcloud.points[i].y = drand48() * 5 + 10;
                    lmekfcloud.points[i].z = 0;
                }
                lmekfcloud.header.stamp = ros::Time::now();
                marker_pub_.publish(lmekfcloud);
            }
            cnt = 0;
        }

        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0,
                                                                              0.0,
                                                                              0.0);
        gimbal_tf_.header.stamp = ros::Time().now();
        gimbal_tf_.transform.rotation = q;
        gimbal_tf_.transform.translation.x = 1;
        gimbal_tf_.transform.translation.y = 1;
        gimbal_tf_.transform.translation.z = 0.15;
        br.sendTransform(gimbal_tf_);

        loop_rate.sleep();
    }
    return 0;
}