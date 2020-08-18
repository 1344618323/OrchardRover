//
// Created by cxn on 2020/6/29.
//

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <rosbag/view.h>

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
    bag.open("test.bag", rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("chatter"));
    topics.push_back(std::string("number"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (rosbag::MessageInstance const m:view) {
        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
        if (s != NULL)
            std::cout << s->data << std::endl;
        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        if (i != NULL)
            std::cout << i->data << std::endl;
    }
    bag.close();
    return 0;
}