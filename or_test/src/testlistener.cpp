#include "ros/ros.h"
#include "std_msgs/String.h"
#include "or_msgs/TrunkObsMsg.h"

// void chatterCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }

void chatterCallback(const or_msgs::TrunkObsMsg::ConstPtr& msg)
{
  for(int i=0;i<msg->ranges.size();i++){
    std::cout<<msg->ranges[i]<<",";
  }
  std::cout<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}