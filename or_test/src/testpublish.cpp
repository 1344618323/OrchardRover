#include "ros/ros.h"
#include "std_msgs/String.h"
#include "or_msgs/TrunkObsMsg.h"
#include <vector>
int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<or_msgs::TrunkObsMsg>("chatter", 1000);
    ros::Publisher chatter_pub2 = n.advertise<or_msgs::TrunkObsMsg>("chatter2", 1000);
    ros::Rate loop_rate(0.5);
    int count = 0;
    while (ros::ok())
    {
        // std_msgs::String msg;
        // std::stringstream ss;
        // ss << "hello world " << count;
        // msg.data = ss.str();
        // ROS_INFO("%s", msg.data.c_str());

        or_msgs::TrunkObsMsg msg;
        std::vector<double> angle_array;
        angle_array.push_back(count);
        // for(int i=0;i<count/10;i++){
        //     angle_array.push_back(count/10+i);
        // }
        msg.header.stamp = ros::Time::now() - ros::Duration(0.1);
        msg.header.frame_id = "base";
        msg.ranges = angle_array;
        msg.bearings = angle_array;
        // msg.angle[0]=10.01;
        // msg.angle[1]=count;
        chatter_pub.publish(msg);

        msg.header.stamp = ros::Time::now();
        msg.ranges = angle_array;
        msg.bearings = angle_array;
        chatter_pub2.publish(msg);

        loop_rate.sleep();
        ++count;
    }
    return 0;
}