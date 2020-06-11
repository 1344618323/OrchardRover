#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_tf_broadcaster");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);
    double count = 0;
    tf::TransformBroadcaster br;

    geometry_msgs::TransformStamped gimbal_tf_;
    gimbal_tf_.header.frame_id = "map";
    gimbal_tf_.child_frame_id = "base";
    while (ros::ok()) {
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0,
                                                                              0.0,
                                                                              0.0);
        gimbal_tf_.header.stamp = ros::Time().now();
        gimbal_tf_.transform.rotation = q;
        gimbal_tf_.transform.translation.x = std::rand() % 100;
        gimbal_tf_.transform.translation.y = 20;
        gimbal_tf_.transform.translation.z = 0.15;
        br.sendTransform(gimbal_tf_);
        loop_rate.sleep();
        count++;
    }
    return 0;
};
