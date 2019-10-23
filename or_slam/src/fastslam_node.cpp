#include "fastslam_node.h"

//s1默认是10hz，720个点

FastSlamNode::FastSlamNode(std::string name)
{
  CHECK(Init()) << "Module " << name << " initialized failed!";
}

bool FastSlamNode::Init()
{
  nh_.param<std::string>("laser_topic_name", laser_topic_, "scan");
  laser_scan_sub_ = nh_.subscribe(laser_topic_, 100, &FastSlamNode::LaserScanCallback, this);
  return true;
}

void FastSlamNode::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg)
{
  LOG_INFO << "Once." << laser_scan_msg->ranges.size()
           << "," << laser_scan_msg->angle_increment
           << "," << laser_scan_msg->angle_increment * laser_scan_msg->ranges.size()
           << "," << laser_scan_msg->time_increment
           << "," << laser_scan_msg->time_increment * laser_scan_msg->ranges.size();
}

int main(int argc, char **argv)
{
  GLogWrapper glog_wrapper(argv[0]);
  ros::init(argc, argv, "fastslam_node");
  FastSlamNode fastslam_node("fastslam_node");
  ros::spin();
  return 0;
}
