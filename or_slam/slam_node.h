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
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <math.h>
#include <vector>

#include <glog/logging.h>

#include "or_msgs/TrunkObsMsgXY.h"
#include "writetxt.h"
#include "types.h"
#include "sim_odom_data_generator.h"
#include "fastslam.h"
#include "fastslam_localization.h"
#include "optimized_slam.h"

#define THREAD_NUM 4 // ROS SPIN THREAD NUM

class SlamNode {
public:
    SlamNode(std::string name);

    ~SlamNode();

    bool Init();

private:
    //ROS Node handle
    ros::NodeHandle nh_;
    bool pure_localization_;

    /**Variables for sim**/
    bool use_sim_;
    std::vector<Eigen::Vector2d> CTrunkPoints_;
    geometry_msgs::Pose ground_truth_pose_;
    ros::Subscriber ground_truth_sub_;
    std::unique_ptr<SimOdomDataGenerator> sim_odom_data_generator_ptr_;
    double trunk_radius_avg_ = 0.1;
    double trunk_radius_sigma_ = 0.02;
    ros::Publisher trunk_obs_pub_;

    /**Variables for laser lidar**/
    double laser_angle_min_;
    double laser_angle_increment_;
    ros::Subscriber laser_scan_sub_;
    std::string laser_topic_;

    /**Variables for visualize**/
    ros::Timer visualize_timer_;
    ros::Publisher landmarks_pub_;
    visualization_msgs::Marker landmarks_msg_;
    ros::Publisher particle_cloud_pub_;
    geometry_msgs::PoseArray particle_cloud_msg_;
    visualization_msgs::Marker node_residual_line_;
    visualization_msgs::Marker lm_residual_line_;

    /**Variables for TF**/
    std::string odom_frame_;
    std::string base_frame_;
    std::string global_frame_;
    std::string gps_frame_;
    std::unique_ptr<tf::TransformListener> tf_listener_ptr_;
    std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_ptr_;
    ros::Duration transform_tolerance_;

    std::string trunk_obs_topic_;
    std::unique_ptr<message_filters::Subscriber<or_msgs::TrunkObsMsgXY>> trunk_obs_sub_;
    std::unique_ptr<tf::MessageFilter<or_msgs::TrunkObsMsgXY>> tf_filter_;
    Vec3d init_pose_;
    Vec3d init_cov_;

    //record data
//    std::unique_ptr<CsvWriter> csv_writer_;
//    std::unique_ptr<FastSlamLocalization> localization_ptr_;
//    std::vector<Vec2d> trunk_obs_vec_;


    //Algorithm object
    std::unique_ptr<optimized_slam::OptimizedSlam> slam_ptr_;

    Vec3d pose_in_odom_;
    ros::Time last_laser_msg_timestamp_;

private:

    void GroundTruthCallbackForSim(const nav_msgs::Odometry::ConstPtr &ground_truth_msg);

    void LaserScanCallbackForSim(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg);

    void TrunkObsMsgCallback(const or_msgs::TrunkObsMsgXY::ConstPtr &trunk_angle_msg);

    bool GetPoseFromTf(const std::string &target_frame,
                       const std::string &source_frame,
                       const ros::Time &timestamp,
                       Vec3d &pose);

    void TransformLaserscanToBaseFrame(double &angle_min,
                                       double &angle_increment,
                                       const sensor_msgs::LaserScan &laser_scan_msg);

    void TimerCallbackForVisualize(const ros::TimerEvent &e);

    bool PublishTf();
};

#endif