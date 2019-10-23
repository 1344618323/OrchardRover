// #include <ros/ros.h>
// #include <tf/transform_listener.h>
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "my_tf_listener");
//     ros::NodeHandle node;
//     ros::Rate loop_rate(10);
//     int count = 0;
//     tf::TransformListener listener;
//     while (ros::ok())
//     {
//         tf::StampedTransform transform;
//         tf::Stamped<tf::Pose> out;
//         try
//         {
//             //issue 1： 想要获取当前时间的变换
//             //直接lookupTransform ros::Time::now()会疯狂ROS_ERROR（catch语句），需要wait
//             // ros::Time now = ros::Time::now();
//             // listener.waitForTransform("map","base",now,ros::Duration(2.0));
//             // listener.lookupTransform("map", "base", now, transform);

//             //issue 2： 想要获取的已有最新变换
//             // listener.lookupTransform("map", "base", ros::Time(0), transform);

//             //issue 3：将数据带入变换
//             tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
//                                                       tf::Vector3(1, 0, 0)),
//                                         ros::Time(0), "base");
//             listener.transformPose("map", ident, out); //亲测，在这里ros::Time()与ros::Time(0)一样
//         }
//         catch (tf::TransformException &ex)
//         {
//             ROS_ERROR("%s", ex.what());
//         }
//         std::cout<<ros::Time::now().sec<<std::endl;
//         // std::cout << "1," << transform.getOrigin().x() << std::endl;
//         std::cout << "2," << out.getOrigin().x() << std::endl;
//         std::cout << ">>>>>>>>>>>>" << std::endl;
//         ++count;
//     }
//     return 0;
// };

//
// Created by cxn on 19-2-15.
//

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include "or_msgs/TrunkAngleMsg.h"

class fuck
{
public:
    fuck()
    {
        point_sub = std::make_shared<message_filters::Subscriber<or_msgs::TrunkAngleMsg>>(nh, "chatter", 5);
        tf_filter = std::make_shared<tf::MessageFilter<or_msgs::TrunkAngleMsg>>(*point_sub, tfl, "map", 1);
        tf_filter->registerCallback(boost::bind(&fuck::msgCallback, this, _1));
    }
    //  Callback to register with tf::MessageFilter to be called when transforms are available
    void msgCallback(const or_msgs::TrunkAngleMsg::ConstPtr &msg)
    {
        tf::StampedTransform transform;
        try
        {
            //issue 2： 想要获取的已有最新变换
            tfl.lookupTransform("map", "base", msg->header.stamp, transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        std::cout << transform.getOrigin().x() << std::endl;
        for (int i = 0; i < msg->angle.size(); i++)
        {
            std::cout << msg->angle[i] << ",";
        }
        std::cout << std::endl;
        std::cout << std::endl;
        // static int cnt;
        // if (cnt++ < 2)
        // {
        //     ros::Rate loop_rate(0.2);
        //     loop_rate.sleep();
        // }
        // if (cnt == 2)
        // {
        //     std::cout << "doneQ" << std::endl;
        // }
    }

private:
    ros::NodeHandle nh;
    tf::TransformListener tfl;
    std::shared_ptr<message_filters::Subscriber<or_msgs::TrunkAngleMsg>> point_sub;
    std::shared_ptr<tf::MessageFilter<or_msgs::TrunkAngleMsg>> tf_filter;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_drawer");
    fuck f;
    ros::spin();
};

//void chatterCallback(const std_msgs::String::ConstPtr &msg) {
//    ROS_INFO("I heard: [%s]", msg->data.c_str());
//    ROS_INFO("%f", ros::Time().now().toSec());
//    ros::Duration(1.0).sleep();
//    ROS_INFO(">>>>>>>>");
//}
//
//int main(int argc, char **argv) {
//    ros::init(argc, argv, "listener");
//    ros::NodeHandle n;
//
//    ros::Subscriber sub1 = n.subscribe("chatter1", 20, chatterCallback);
//    ros::Subscriber sub2 = n.subscribe("chatter2", 20, chatterCallback);
////    ros::spin();
//    ros::AsyncSpinner async_spinner(2);
//    async_spinner.start();
//    ros::waitForShutdown();
//    return 0;
//}
