#ifndef OR_PFLOCALIZATION_ALG_H
#define OR_PFLOCALIZATION_ALG_H

#include "fastslam.h"
#include <writetxt.h>

class PfLocalization : public FastSlam {
public:
    PfLocalization(const Vec3d &init_pose, const Vec3d &init_cov, ros::NodeHandle *nh,
                   const std::vector<Vec2d> land_marks);

    int Update(const Vec3d &pose,
               const std::vector<Vec2d> zs, geometry_msgs::PoseArray &particle_cloud_pose_msg);

protected:

    void UpdateObserveData(const std::vector<Vec2d> zs);

    void
    GetParticlesCloudMsg(geometry_msgs::PoseArray &particle_cloud_pose_msg);

private:

    double ComputeParticleWeight(const Vec3d &sensor_pose, const Vec2d &z, const Mat2d &Q, const Mat2d &invQ);

    double ComputeParticleWeightForTest(const Vec3d &sensor_pose, const Vec2d &z,
                                        const Mat2d &Q, const Mat2d &invQ);

    void UpdateObserveDataForTest(const std::vector<Vec2d> zs);

    //record data
    std::unique_ptr<CsvWriter> csv_writer_;

    std::vector<Vec2d> land_marks_;

    Mat2d invQ_;
};


#endif
