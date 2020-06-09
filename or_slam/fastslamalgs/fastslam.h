#ifndef OR_FASTSLAM_ALG_H
#define OR_FASTSLAM_ALG_H

#include <vector>

#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>

// #include "log.h"
#include "types.h"

#include "fastslam_config.h"
#include "sensors/sensor_odom.h"
#include "particle_filter/particle_filter.h"
#include "../include/types.h"
#include <visualization_msgs/Marker.h>
#include "writetxt.h"
class FastSlam
{
public:
    FastSlam(const Vec3d &init_pose, const Vec3d &init_cov, ros::NodeHandle *nh, bool multi_sensor);

    ~FastSlam();

    int Update(const Vec3d &pose, const std::vector<Vec2d> &zs,
               geometry_msgs::PoseArray &particle_cloud_pose_msg,
               visualization_msgs::Marker &lm_cloud_msg);

    void SetSensorPose(const Vec3d &sensor_pose);

    void SetMultiSensorPose(const Vec3d &sensor_pose1, const Vec3d &sensor_pose2);

    /********20.1.3*******/
    Vec3d gus_pose;
    std::vector<PfLandMark> gus_lm_vec;

protected:
    void UpdateOdomPoseData(const Vec3d &pose);

    void UpdateObserveData(const std::vector<Vec2d> &zs);

    void ComputeJaccobians(const Vec3d &sensor_pose, const PfLandMark &lm,
                           const Vec2d &z, const Mat2d &Q,
                           Mat2d &Hj, Mat2d &Qj, Vec2d &dz);

    double ComputeWeight(const Mat2d &Qj, const Vec2d &dz);

    double ComputeMahalanobisDis(const Mat2d &Qj, const Vec2d &dz);

    void AddNewLd(const Vec3d &sensor_pose, const Vec2d &z, const Mat2d &Q,
                  PfLandMark &new_lm);

    void DataAssociate(const Vec3d &sensor_pose, const ParticleFilterSample &sample,
                       const Vec2d &z, const Mat2d &Q,
                       double &max_w, int &max_i);

    void UpdateKfByCholesky(PfLandMark &lm, const Vec2d &dz, const Mat2d &Q, const Mat2d &Hj);

    void GetParticlesCloudMsg(geometry_msgs::PoseArray &particle_cloud_pose_msg,
                              visualization_msgs::Marker &lm_cloud_msg);

protected:
    Vec3d init_pose_;
    Mat3d init_cov_;

    int particles_num_;

    double odom_alpha1_;
    double odom_alpha2_;
    double odom_alpha3_;
    double odom_alpha4_;

    double update_min_d_; //d_thresh_
    double update_min_a_; //a_thresh_

    Mat2d observe_cov_;

    double new_ld_mahalanobis_dis = 300;
    double min_weight = 1e-20;

    //每次update函数最后置为false，若里程计测到的位移大于阈值，就不会专门置为true，
    bool odom_update_ = false;

    std::unique_ptr<SensorOdom> odom_model_ptr_;

    ParticleFilterPtr pf_ptr_;
    bool pf_init_;
    Vec3d pf_odom_pose_;
    Vec3d sensor_pose_;
    Vec3d multi_sensor_pose_[2];
    bool multi_sensor_sign_;

    //record data
    std::unique_ptr<CsvWriter> csv_writer_;
    std::vector<Vec2d> true_lms_;

    double mindistance(Vec2d x)
    {
        double min = 1000000;
        for (int i = 0; i < true_lms_.size(); i++)
        {
            double dis=(true_lms_[i]-x).norm();
            if (dis < min)
            {
                min = dis;
            }
        }
        return min;
    }
};

#endif
