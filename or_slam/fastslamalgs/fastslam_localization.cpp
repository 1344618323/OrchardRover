#include "fastslam_localization.h"

FastSlamLocalization::FastSlamLocalization(const Vec3d &init_pose, const Vec3d &init_cov,
                                           ros::NodeHandle *nh, const std::vector<Vec2d> &land_marks,
                                           bool multi_sensor) : FastSlam(init_pose, init_cov, nh, multi_sensor) {
    land_marks_ = land_marks;

    bool invertible;
    observe_cov_.computeInverseWithCheck(invQ_, invertible);
    if (!invertible) {
        return;
    }

    std::vector<std::string> csv_topic = {"mind", "mindsec", "midthrid"};
    csv_writer_ = std::make_unique<CsvWriter>("/home/cxn/data.csv", csv_topic);
}

int FastSlamLocalization::Update(const Vec3d &pose,
                                 const std::vector<Vec2d> &zs, geometry_msgs::PoseArray &particle_cloud_pose_msg) {
    UpdateOdomPoseData(pose);
    if (zs.size() != 0 && odom_update_) {
        UpdateObserveData(zs);
        if (pf_ptr_->UpdateResample())
            UpdateObserveData(zs);
    }
    GetParticlesCloudMsg(particle_cloud_pose_msg);
    return 0;
}

void FastSlamLocalization::UpdateObserveData(const std::vector<Vec2d> &zs) {
    SampleSetPtr set_ptr = pf_ptr_->GetCurrentSampleSetPtr();

    std::cout << "!!!!!!!!!!! Update observe data: " << zs.size() << " !!!!!!!!!!!" << std::endl;

    for (int N = 0; N < zs.size(); N++) {

        // 如果测量范围不对,则跳过
        if (zs[N](0) <= 0.05 || zs[N](0) >= 8)
            continue;

        std::cout << "<<<<<<<<<< zs " << N << " <<<<<<<<<<" << std::endl;
        for (int k = 0; k < set_ptr->sample_count; k++) {
            ParticleFilterSample &sample = set_ptr->samples_vec[k];

            Vec3d sensor_pose;
            if (multi_sensor_sign_) {
                sensor_pose = CoordAdd(sample.pose, multi_sensor_pose_[N]);
            } else {
                //获得雷达坐标
                sensor_pose = CoordAdd(sample.pose, sensor_pose_);
            }

            double weight = ComputeParticleWeight(sensor_pose, zs[N], observe_cov_, invQ_);

            if (sample.weight * weight > min_weight)
                sample.weight *= weight;

            std::cout << "sample index: " << k << ", weight:" << sample.weight << std::endl;
        }
    }
}

double
FastSlamLocalization::ComputeParticleWeight(const Vec3d &sensor_pose, const Vec2d &z,
                                            const Mat2d &Q, const Mat2d &invQ) {
    std::vector<double> mah_dis_vec;
    for (int j = 0; j < land_marks_.size(); j++) {
        Vec2d dz;

        // 粒子中地标与粒子坐标的差
        Vec2d dxy = land_marks_[j] - sensor_pose.segment(0, 2);
        double d2 = dxy(0) * dxy(0) + dxy(1) * dxy(1);
        double d = sqrt(d2);

        Vec2d z_hat;
        z_hat(0) = d;
        z_hat(1) = AngleDiff<double>(atan2(dxy(1), dxy(0)), sensor_pose(2));

        dz << z(0) - z_hat(0), AngleDiff<double>(z(1), z_hat(1));

        double mahalanobis_dis = dz.transpose() * invQ * dz;
        mah_dis_vec.push_back(mahalanobis_dis);
    }

    double min_dis = FLT_MAX;
    // obtain the max weight
    for (int j = 0; j < mah_dis_vec.size(); j++) {
        if (mah_dis_vec[j] < min_dis && mah_dis_vec[j] >= 0) {
            min_dis = mah_dis_vec[j];
        }
    }

    double max_w = 1;
    if (min_dis < FLT_MAX)
        max_w = exp(-0.5 * min_dis) / sqrt(Q.determinant());

    /**************** For test *****************/
    double min_dis_sec = FLT_MAX;
    for (int j = 0; j < mah_dis_vec.size(); j++) {
        if (mah_dis_vec[j] < min_dis_sec && mah_dis_vec[j] > min_dis) {
            min_dis_sec = mah_dis_vec[j];
        }
    }

    double min_dis_thrid = FLT_MAX;
    for (int j = 0; j < mah_dis_vec.size(); j++) {
        if (mah_dis_vec[j] < min_dis_thrid && mah_dis_vec[j] > min_dis_sec) {
            min_dis_thrid = mah_dis_vec[j];
        }
    }


    csv_writer_->write(min_dis);
    csv_writer_->write(min_dis_sec);
    csv_writer_->write(min_dis_thrid);


    return max_w;


}

void FastSlamLocalization::GetParticlesCloudMsg(geometry_msgs::PoseArray &particle_cloud_pose_msg) {
    particle_cloud_pose_msg.poses.resize(pf_ptr_->GetCurrentSampleSetPtr()->sample_count);

    for (int i = 0; i < pf_ptr_->GetCurrentSampleSetPtr()->sample_count; i++) {

        ParticleFilterSample &sample = pf_ptr_->GetCurrentSampleSetPtr()->samples_vec[i];
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(sample.pose[2]),
                                 tf::Vector3(sample.pose[0],
                                             sample.pose[1],
                                             0)),
                        particle_cloud_pose_msg.poses[i]);
    }
}