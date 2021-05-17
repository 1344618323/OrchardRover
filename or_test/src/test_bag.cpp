//
// Created by cxn on 2020/6/29.
//

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <rosbag/view.h>
#include "geometry_msgs/Twist.h"
#include <fstream>

//往bag里写数据
//int main(int argc, char **argv) {
//    ros::init(argc, argv, "rosbag_test");
//    ros::NodeHandle n;
//    rosbag::Bag bag;
//    bag.open("test.bag", rosbag::bagmode::Write);
//
//    int count = 0;
//    ros::Rate r(5);
//    while (ros::ok()) {
//        std::stringstream ss;
//        ss << "foo" << count;
//
//        std_msgs::String str;
//        str.data = ss.str();
//
//        std_msgs::Int32 i;
//        i.data = count;
//
//        bag.write("chatter", ros::Time::now(), str);
//        bag.write("number", ros::Time::now(), i);
//
//        count++;
//        r.sleep();
//    }
//    bag.close();
//    return 0;
//}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbag_test");
    ros::NodeHandle n;
    rosbag::Bag bag;
    bag.open("/home/cxn/myfile/OR_ws2/2021-05-04-14-59-56.bag", rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/cmd_vel"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::ofstream csv_file_;
    csv_file_.open("/home/cxn/myfile/OR_ws/orchardrover_ws/data.csv", std::ios::out);

    for (rosbag::MessageInstance const m:view) {
        geometry_msgs::Twist::ConstPtr vel = m.instantiate<geometry_msgs::Twist>();
        if (vel != NULL) {
            std::cout << vel->linear.x << std::endl;
            std::cout << vel->angular.z << std::endl;

            csv_file_ << vel->linear.x << ','
                      << vel->angular.z << std::endl;
        }
    }

    csv_file_.close();

    bag.close();
    return 0;
}