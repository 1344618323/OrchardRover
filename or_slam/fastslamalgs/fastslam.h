#ifndef OR_FASTSLAM_ALG_H
#define OR_FASTSLAM_ALG_H

#include <vector>

#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>

#include "log.h"
#include "types.h"

#include "fastslam_config.h"
#include "sensors/sensor_odom.h"
#include "particle_filter/particle_filter.h"


class FastSlam {
public:
    FastSlam(const Vec3d &init_pose, const Vec3d &init_cov, ros::NodeHandle *nh);

    ~FastSlam();

    int Update(const Vec3d &pose,
               const std::vector<Vec2d> land_marks);

    void SetSensorPose(const Vec3d &sensor_pose);

private:

    void UpdateOdomPoseData(const Vec3d pose);

    void UpdateObserveData(const std::vector<Vec2d> zs);

    void ComputeJaccobians(const Vec3d &sensor_pose, const Vec2d &lm_pose, const Mat2d &lm_cov,
                           const Vec2d &z, const Mat2d &Q, Mat2d &Hj, Mat2d &Qj,
                           Vec2d &dz);

    double ComputeWeight(const Mat2d &Qj, const Vec2d &dz);

    void AddNewLd(const Vec3d &sensor_pose, const Vec2d &z, const Mat2d &Q, Vec2d &new_lm_pose, Mat2d &new_lm_cov);

    void DataAssociate(const Vec3d &sensor_pose, const ParticleFilterSample &sample, const Vec2d &z, const Mat2d &Q,
                       double &max_w, int &max_i);

    //def update_KF_with_cholesky(xf, Pf, v, Q, Hf):
    void UpdateKFwithCholesky(Vec2d &lm_pose, Mat2d &lm_cov, const Vec2d &dz, const Mat2d &Q, const Mat2d &Hj);

    void NormalizeWeight(const SampleSetPtr &set_ptr);

    void Resample(const SampleSetPtr &set_ptr);

private:
    Vec3d init_pose_;
    Mat3d init_cov_;

    int particles_num_;

//  OdomModel odom_model = ODOM_MODEL_OMNI;
    double odom_alpha1_;
    double odom_alpha2_;
    double odom_alpha3_;
    double odom_alpha4_;


    double update_min_d_;//d_thresh_
    double update_min_a_;//a_thresh_

    Mat2d observe_cov_;
    double p0;

    //每次update函数最后置为false，若里程计测到的位移大于阈值，就不会专门置为true，
    bool odom_update_ = true;

    bool resampled_ = false;

    std::unique_ptr<SensorOdom> odom_model_ptr_;

    std::unique_ptr<ParticleFilter> pf_ptr_;
    bool pf_init_;
    Vec3d pf_odom_pose_;

    Vec3d sensor_pose_;
};


#endif
