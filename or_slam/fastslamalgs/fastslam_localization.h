#ifndef OR_FASTSLAM_LOCALIZATION_ALG_H
#define OR_FASTSLAM_LOCALIZATION_ALG_H

#include "fastslam.h"
#include <writetxt.h>

class FastSlamLocalization : public FastSlam {
public:
    FastSlamLocalization(const Vec3d &init_pose, const Vec3d &init_cov, ros::NodeHandle *nh,
                         const std::vector<Vec2d> &land_marks,bool multi_sensor);

    int Update(const Vec3d &pose, const std::vector<Vec2d> &zs,
               geometry_msgs::PoseArray &particle_cloud_pose_msg);

private:

    void UpdateObserveData(const std::vector<Vec2d> &zs);

    void GetParticlesCloudMsg(geometry_msgs::PoseArray &particle_cloud_pose_msg);

    double ComputeParticleWeight(const Vec3d &sensor_pose, const Vec2d &z,
                                 const Mat2d &Q, const Mat2d &invQ);

    //record data
    std::unique_ptr<CsvWriter> csv_writer_;

    std::vector<Vec2d> land_marks_;
    Mat2d invQ_;
};


#endif
