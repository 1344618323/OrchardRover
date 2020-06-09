#ifndef OR_OPTIMIZED_SLAM_ALG_H
#define OR_OPTIMIZED_SLAM_ALG_H

#include <ros/ros.h>
#include <chrono>
#include <limits>
#include <ceres/ceres.h>
#include "optimized_slam_config.h"
#include "transform/rigid_transform.h"
#include "cost_function/spa_cost_function_2d.h"

namespace optimizedSlam {
    struct NodeSpec2D {
        ros::Time time;
        transform::Rigid2d odom_pose_2d;
        transform::Rigid2d global_pose_2d;
    };

    class OptimizedSlam {
    public:
        OptimizedSlam(const Eigen::Vector3d &init_pose, ros::NodeHandle *nh);

        ~OptimizedSlam();

        void AddNodeData(const Eigen::Vector3d &ros_odom_pose, const std::vector<double> &XYs, const ros::Time &stamp);

        void SetSensorPose(const Eigen::Vector3d &sensor_pose);

        const std::map<int, Eigen::Vector3d> GetLandmarks();

        const Eigen::Vector3d GetPose(const Eigen::Vector3d &odom_pose);

    private:
        void Solve();

        void CalculateLikelihood(std::map<int, NodeSpec2D>::iterator &it, Eigen::Vector2d &xy);

        std::array<double, 3> FromPose(const transform::Rigid2d &pose);

        Eigen::Vector3d EigenV3FromPose(const transform::Rigid2d &pose);

        transform::Rigid2d ToPose(const std::array<double, 3> &values);

    private:
        std::map<int, NodeSpec2D> node_data_;//每进入一个node，就给first+1
        std::map<int, optimization::LandmarkNode> landmark_data_;//每新加一个lm，就给first+1

        Eigen::Vector3d last_ros_odom_pose_;
        transform::Rigid2d init_global_pose_;

        double update_min_d_ = 0.2;
        double update_min_a_ = 0.2;
        bool algs_init_ = false;
        Eigen::Matrix2d cov_z_inv_;

        double odom_translation_weight_;
        double odom_rotation_weight_;
        double lm_translation_weight_;
        double lm_rotation_weight_;

    };
}
#endif
