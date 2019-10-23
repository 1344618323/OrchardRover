#ifndef ORCHARDROVER_FASTSLAM_NODE_H
#define ORCHARDROVER_FASTSLAM_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>

#include "log.h"
#include "or_msgs/TrunkAngleMsg.h"

#define THREAD_NUM 4 // ROS SPIN THREAD NUM

class FastSlamNode
{
public:
    FastSlamNode(std::string name);
    bool Init();
    void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg);
    // void PublishVisualize();

private:
    //ROS Node handle
    ros::NodeHandle nh_;
    ros::Subscriber laser_scan_sub_;
    std::string laser_topic_;
};

#endif