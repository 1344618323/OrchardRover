#ifndef ORCHARDROVER_SLAM_NODE_H
#define ORCHARDROVER_SLAM_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include "log.h"
#include "or_msgs/TrunkObsMsg.h"
#include "writetxt.h"
#include <vector>
#include "types.h"
#include <visualization_msgs/Marker.h>
#include "fastslam.h"
#include "pflocalization.h"

#define THREAD_NUM 4 // ROS SPIN THREAD NUM


class SlamNode {
public:
    SlamNode(std::string name);

    bool Init();

private:
    //ROS Node handle
    ros::NodeHandle nh_;

    bool pure_localization_;

    ros::Subscriber laser_scan_sub_;
    std::string laser_topic_;
    std::string laser_topic_1_;

    std::unique_ptr<tf::TransformListener> tf_listener_ptr_;
    std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_ptr_;

    std::unique_ptr<message_filters::Subscriber<or_msgs::TrunkObsMsg>> trunk_obs_sub_;
    std::unique_ptr<tf::MessageFilter<or_msgs::TrunkObsMsg>> tf_filter_;
    std::string trunk_obs_topic_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string global_frame_;
    ros::Duration transform_tolerance_;
    Vec3d init_pose_;
    Vec3d init_cov_;

    std::vector<sensor_msgs::LaserScan> laser_msg_queue_;

    //record data
    std::unique_ptr<CsvWriter> csvWriter_;


    std::vector<Vec2d> trunk_obs_vec_;
    //Algorithm object
    std::unique_ptr<FastSlam> slam_ptr_;
    std::unique_ptr<PfLocalization> localization_ptr_;

    ros::Publisher sim_trunk_obs_pub_;
    double laser_angle_min_;
    double laser_angle_increment_;

    ros::Publisher particlecloud_pub_;
    geometry_msgs::PoseArray particlecloud_msg_;
    ros::Publisher lmcloud_pub_;
    visualization_msgs::Marker lmcloud_msg_;


    double trunk_radius_avg_ = 0.1;
    double trunk_radius_sigma_ = 0.02;
private:
    void GetLaserPose();

    sensor_msgs::LaserScan ChooseLaserScan(const or_msgs::TrunkObsMsg::ConstPtr &trunk_angle_msg);

    bool GetPoseFromTf(const std::string &target_frame,
                       const std::string &source_frame,
                       const ros::Time &timestamp,
                       Vec3d &pose);

    void GetTrunkPosition(const sensor_msgs::LaserScan &laser_scan_msg,
                          const or_msgs::TrunkObsMsg::ConstPtr &trunk_angle_msg);

    void GetTrunkPosition(const or_msgs::TrunkObsMsg::ConstPtr &trunk_angle_msg);

    void TransformLaserscanToBaseFrame(double &angle_min,
                                       double &angle_increment,
                                       const sensor_msgs::LaserScan &laser_scan_msg);

    void PublishVisualize();

    bool PublishTf();


    void LaserScanCallbackForCheck(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg);

    void LaserScanCallbackForSim(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg);

    void LaserScanCallbackForSave(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg);

    void TrunkObsMsgCallback(const or_msgs::TrunkObsMsg::ConstPtr &trunk_angle_msg);
};

#endif