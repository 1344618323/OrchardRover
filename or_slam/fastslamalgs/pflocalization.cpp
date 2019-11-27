#include "pflocalization.h"

PfLocalization::PfLocalization(const Vec3d &init_pose, const Vec3d &init_cov, ros::NodeHandle *nh,
                               const std::vector<Vec2d> land_marks) : FastSlam(init_pose, init_cov, nh)
{
    land_marks_ = land_marks;

    bool invertible;
    observe_cov_.computeInverseWithCheck(invQ_, invertible);
    if (!invertible)
    {
        return;
    }

    std::vector<std::string> csv_topic = {"mind", "minsecd", "maxw", "maxsecw"};
    csv_writer_ = std::make_unique<CsvWriter>("/home/cxn/data.csv", csv_topic);
}

int PfLocalization::Update(const Vec3d &pose,
                           const std::vector<Vec2d> zs, geometry_msgs::PoseArray &particle_cloud_pose_msg)
{
    UpdateOdomPoseData(pose);
    if (zs.size() != 0 && odom_update_)
    {
        UpdateObserveData(zs);
        if (pf_ptr_->UpdateResample())
            UpdateObserveDataForTest(zs);
    }
    GetParticlesCloudMsg(particle_cloud_pose_msg);
    return 0;
}

void PfLocalization::UpdateObserveData(const std::vector<Vec2d> zs)
{
    SampleSetPtr set_ptr = pf_ptr_->GetCurrentSampleSetPtr();

    std::cout << "!!!!!!!!!!! Update observe data: " << zs.size() << " !!!!!!!!!!!" << std::endl;

    for (int N = 0; N < zs.size(); N++)
    {

        std::cout << "<<<<<<<<<< zs " << N << " <<<<<<<<<<" << std::endl;
        for (int k = 0; k < set_ptr->sample_count; k++)
        {
            ParticleFilterSample &sample = set_ptr->samples_vec[k];
            //获得雷达坐标
            Vec3d sensor_pose = CoordAdd(sample.pose, sensor_pose_);

            double weight = ComputeParticleWeight(sensor_pose, zs[N], observe_cov_, invQ_);

            sample.weight *= weight;

            std::cout << "sample index: " << k << ", " << sample.weight << std::endl;
        }
    }
}

double
PfLocalization::ComputeParticleWeight(const Vec3d &sensor_pose, const Vec2d &z,
                                      const Mat2d &Q, const Mat2d &invQ)
{
    std::vector<double> weights;
    std::vector<double> ms;

    for (int j = 0; j < land_marks_.size(); j++)
    {
        Vec2d dz;

        // 粒子中地标与粒子坐标的差
        Vec2d dxy = land_marks_[j] - sensor_pose.segment(0, 2);
        double d2 = dxy(0) * dxy(0) + dxy(1) * dxy(1);
        double d = sqrt(d2);

        Vec2d z_hat;
        z_hat(0) = d;
        z_hat(1) = angle_diff<double>(atan2(dxy(1), dxy(0)), sensor_pose(2));

        dz << z(0) - z_hat(0), angle_diff<double>(z(1), z_hat(1));

        double mahalanobis_dis = dz.transpose() * invQ * dz;

        double weight = exp(-0.5 * mahalanobis_dis) / sqrt(Q.determinant());

        weights.push_back(weight);

        ms.push_back(mahalanobis_dis);
    }

    double max_w = 0;
    // obtain the max weight
    for (int j = 0; j < weights.size(); j++)
    {
        if (weights[j] > max_w)
        {
            max_w = weights[j];
        }
    }

    return max_w;
}

void PfLocalization::UpdateObserveDataForTest(const std::vector<Vec2d> zs)
{
    SampleSetPtr set_ptr = pf_ptr_->GetCurrentSampleSetPtr();

    for (int N = 0; N < zs.size(); N++)
    {

        for (int k = 0; k < set_ptr->sample_count; k++)
        {
            ParticleFilterSample &sample = set_ptr->samples_vec[k];
            //获得雷达坐标
            Vec3d sensor_pose = CoordAdd(sample.pose, sensor_pose_);

            double weight = ComputeParticleWeightForTest(sensor_pose, zs[N], observe_cov_, invQ_);

            sample.weight *= weight;
        }
    }
}

double
PfLocalization::ComputeParticleWeightForTest(const Vec3d &sensor_pose, const Vec2d &z,
                                             const Mat2d &Q, const Mat2d &invQ)
{
    std::vector<double> weights;
    std::vector<double> ms;

    for (int j = 0; j < land_marks_.size(); j++)
    {
        Vec2d dz;

        // 粒子中地标与粒子坐标的差
        Vec2d dxy = land_marks_[j] - sensor_pose.segment(0, 2);
        double d2 = dxy(0) * dxy(0) + dxy(1) * dxy(1);
        double d = sqrt(d2);

        Vec2d z_hat;
        z_hat(0) = d;
        z_hat(1) = angle_diff<double>(atan2(dxy(1), dxy(0)), sensor_pose(2));

        dz << z(0) - z_hat(0), angle_diff<double>(z(1), z_hat(1));

        double mahalanobis_dis = dz.transpose() * invQ * dz;

        double weight = exp(-0.5 * mahalanobis_dis) / sqrt(Q.determinant());

        weights.push_back(weight);

        ms.push_back(mahalanobis_dis);
    }

    double min_d = FLT_MAX;
    int min_d_i = 0;
    // obtain the max weight
    for (int j = 0; j < ms.size(); j++)
    {
        if (ms[j] < min_d)
        {
            min_d = ms[j];
            min_d_i = j;
        }
    }
    double sec_d = FLT_MAX;
    for (int j = 0; j < ms.size(); j++)
    {
        if (ms[j] < sec_d && j != min_d_i)
        {
            sec_d = ms[j];
        }
    }

    double max_w = 0;
    int max_i = 0;
    // obtain the max weight
    for (int j = 0; j < weights.size(); j++)
    {
        if (weights[j] > max_w)
        {
            max_w = weights[j];
            max_i = j;
        }
    }
    double sec_w = 0;
    for (int j = 0; j < weights.size(); j++)
    {
        if (weights[j] > sec_w && j != max_i)
        {
            sec_w = weights[j];
        }
    }

    csv_writer_->write(min_d);
    csv_writer_->write(sec_d);
    csv_writer_->write(max_w);
    csv_writer_->write(sec_w);

    return max_w;
}

void PfLocalization::GetParticlesCloudMsg(geometry_msgs::PoseArray &particle_cloud_pose_msg)
{
    particle_cloud_pose_msg.poses.resize(pf_ptr_->GetCurrentSampleSetPtr()->sample_count);

    for (int i = 0; i < pf_ptr_->GetCurrentSampleSetPtr()->sample_count; i++)
    {

        ParticleFilterSample &sample = pf_ptr_->GetCurrentSampleSetPtr()->samples_vec[i];
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(sample.pose[2]),
                                 tf::Vector3(sample.pose[0],
                                             sample.pose[1],
                                             0)),
                        particle_cloud_pose_msg.poses[i]);
    }
}