#include "fastslam.h"

FastSlam::FastSlam(const Vec3d &init_pose, const Vec3d &init_cov, ros::NodeHandle *nh) {
    nh->param<int>("particles_num", particles_num_, 500);
    nh->param<double>("odom_alpha1", odom_alpha1_, 0.005);
    nh->param<double>("odom_alpha2", odom_alpha2_, 0.005);
    nh->param<double>("odom_alpha3", odom_alpha3_, 0.01);
    nh->param<double>("odom_alpha4", odom_alpha4_, 0.005);
    nh->param<double>("update_min_d", update_min_d_, 0.2); //在执行滤波更新前平移运动的距离0.05(50mm)与0.03（1.7度）
    nh->param<double>("update_min_a", update_min_a_, 0.5);

    observe_cov_.setZero();
    nh->param<double>("observe_cov_distance", observe_cov_(0, 0), 0);
    nh->param<double>("observe_cov_bearing", observe_cov_(1, 1), 0);

    odom_model_ptr_ = std::make_unique<SensorOdom>(odom_alpha1_,
                                                   odom_alpha2_,
                                                   odom_alpha3_,
                                                   odom_alpha4_);

    init_pose_ = init_pose;
    init_cov_.setZero();
    init_cov_(0, 0) = init_cov(0);
    init_cov_(0, 0) = init_cov(1);
    init_cov_(0, 0) = init_cov(2);
    pf_ptr_ = std::make_shared<ParticleFilter>(particles_num_, init_pose_, init_cov_);
    pf_init_ = false;

    LOG_INFO << "FastSlam Init!";
}

FastSlam::~FastSlam() {
    LOG_INFO << "FastSlam Delete!";
}

int FastSlam::Update(const Vec3d &pose, const std::vector<Vec2d> land_marks,
                     geometry_msgs::PoseArray &particle_cloud_pose_msg,
                     visualization_msgs::Marker &lm_cloud_msg) {
    UpdateOdomPoseData(pose);
    if (land_marks.size() != 0 && odom_update_) {
        UpdateObserveData(land_marks);
        //resample
        pf_ptr_->UpdateResample();
    }
    GetParticlesCloudMsg(particle_cloud_pose_msg, lm_cloud_msg);
    return 0;
}

void FastSlam::GetParticlesCloudMsg(geometry_msgs::PoseArray &particle_cloud_pose_msg,
                                    visualization_msgs::Marker &lm_cloud_msg) {
    particle_cloud_pose_msg.poses.resize(pf_ptr_->GetCurrentSampleSetPtr()->sample_count);

    lm_cloud_msg.points.clear();

    for (int i = 0; i < pf_ptr_->GetCurrentSampleSetPtr()->sample_count; i++) {

        ParticleFilterSample &sample = pf_ptr_->GetCurrentSampleSetPtr()->samples_vec[i];
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(sample.pose[2]),
                                 tf::Vector3(sample.pose[0],
                                             sample.pose[1],
                                             0)),
                        particle_cloud_pose_msg.poses[i]);

        for (int j = 0; j < sample.landmark_num; j++) {
            geometry_msgs::Point temp;
            temp.x = sample.lm_poses[j](0);
            temp.y = sample.lm_poses[j](1);
            temp.z = 0;
            lm_cloud_msg.points.push_back(temp);
        }
    }
}

void FastSlam::UpdateOdomPoseData(const Vec3d pose) {
    Vec3d delta;
    delta.setZero();
    if (!pf_init_) {
        // Pose at last filter update
        pf_odom_pose_ = pose;
        // Filter is now initialized"
        pf_init_ = true;
        // Set update sensor data flag
        odom_update_ = true;
    }     // If the robot has moved, update the filter
    else if (pf_init_) {
        // Compute change in pose
        delta[0] = pose[0] - pf_odom_pose_[0];
        delta[1] = pose[1] - pf_odom_pose_[1];
        delta[2] = angle_diff<double>(pose[2], pf_odom_pose_[2]);

        // See if we should update the filter
        odom_update_ = std::fabs(delta[0]) > update_min_d_ ||
                       std::fabs(delta[1]) > update_min_d_ ||
                       std::fabs(delta[2]) > update_min_a_;

        if (odom_update_) {
            DLOG_INFO << "Robot has moved, update the filter";
            SensorOdomData odom_data;
            odom_data.pose = pose;
            odom_data.delta = delta;
            odom_model_ptr_->UpdateAction(pf_ptr_->GetCurrentSampleSetPtr(), odom_data);
            pf_odom_pose_ = pose;
        }
    }
}


void FastSlam::SetSensorPose(const Vec3d &sensor_pose) {
    sensor_pose_ = sensor_pose;
    DLOG_INFO << "sensor pose: " <<
              sensor_pose_[0] << ", " <<
              sensor_pose_[1] << ", " <<
              sensor_pose_[2];
}


void FastSlam::UpdateObserveData(const std::vector<Vec2d> zs) {
    SampleSetPtr set_ptr = pf_ptr_->GetCurrentSampleSetPtr();

    std::cout << "!!!!!!!!!!! Update observe data: " << zs.size() << " !!!!!!!!!!!" << std::endl;

    for (int N = 0; N < zs.size(); N++) {

        std::cout << "<<<<<<<<<< zs " << N << " <<<<<<<<<<" << std::endl;
        for (int k = 0; k < set_ptr->sample_count; k++) {
            ParticleFilterSample &sample = set_ptr->samples_vec[k];
            //获得雷达坐标
            Vec3d sensor_pose = CoordAdd(sample.pose, sensor_pose_);

            std::cout << "sample index: " << k << std::endl;
            std::cout << "sensor pose: " << sensor_pose[0] << "," << sensor_pose[1] << ","
                      << sensor_pose[2] << ", sample pose: " << sample.pose[0] << "," << sample.pose[1] << ","
                      << sample.pose[2] << std::endl;

            double max_w;
            int max_i;

            DataAssociate(sensor_pose, sample, zs[N], observe_cov_, max_w, max_i);

            // update the weight of particle
            sample.weight *= max_w;

            if (max_i == sample.landmark_num) {
                //new landmarks
                Vec2d new_ld_pose;
                Mat2d new_ld_cov;
                AddNewLd(sensor_pose, zs[N], observe_cov_, new_ld_pose, new_ld_cov);
                sample.lm_poses.push_back(new_ld_pose);
                sample.lm_covs.push_back(new_ld_cov);
                sample.lm_cnt.push_back(1);
                sample.landmark_num++;

                std::cout << "sample " << k << " add a new landmark: " <<
                          new_ld_pose[0] << "," << new_ld_pose[1] << std::endl;

            } else {
                //update EKF
                Mat2d Hj;
                Mat2d Qj;
                Vec2d dz;
                ComputeJaccobians(sensor_pose, sample.lm_poses[max_i], sample.lm_covs[max_i],
                                  zs[N], observe_cov_, Hj, Qj, dz);
                UpdateKFwithCholesky(sample.lm_poses[max_i], sample.lm_covs[max_i], dz, observe_cov_, Hj);
                sample.lm_cnt[max_i]++;
            }
        }
    }
}

/*
 * sensor_pose 雷达坐标
 * lm_pose 地标坐标
 * lm_cov 地标坐标协方差
 * z 观测量
 * Q 传感器测量协方差
 * Hj 观测函数对地标坐标的导数
 * Qj 观测协方差
 * dz 真实观测量与假设观测量之差
 */
void
FastSlam::ComputeJaccobians(const Vec3d &sensor_pose, const Vec2d &lm_pose, const Mat2d &lm_cov,
                            const Vec2d &z, const Mat2d &Q, Mat2d &Hj, Mat2d &Qj,
                            Vec2d &dz) {
    // 粒子中地标与粒子坐标的差
    Vec2d dxy = lm_pose - sensor_pose.segment(0, 2);
    double d2 = dxy(0) * dxy(0) + dxy(1) * dxy(1);
    double d = sqrt(d2);

    Vec2d z_hat;
    z_hat(0) = d;
    z_hat(1) = angle_diff<double>(atan2(dxy(1), dxy(0)), sensor_pose(2));

    //Hj 是观测函数h对地标位置(x,y)的偏导
    Hj << dxy(0) / d, dxy(1) / d,
            -dxy(1) / d2, dxy(0) / d2;

    Qj = Hj * lm_cov * Hj.transpose() + Q;
    dz << z(0) - z_hat(0), angle_diff<double>(z(1), z_hat(1));
}


/*
 * Qj 观测协方差
 * dz 真实观测量与假设观测量之差
 * 输出权重
 */
double FastSlam::ComputeWeight(const Mat2d &Qj, const Vec2d &dz) {
    bool invertible;
    Mat2d invQj;
    Qj.computeInverseWithCheck(invQj, invertible);
    if (!invertible) {
        return -1;
    }
    double mahalanobis_dis = dz.transpose() * invQj * dz;

    std::cout << "!!!!!!!!!!!!!!!" << mahalanobis_dis << "!!!!!!!!!!!!!!!" << std::endl;

    double weight = exp(-0.5 * mahalanobis_dis) / sqrt(fabs(2 * M_PI * Qj.determinant()));
    assert(weight >= 0);

    return weight;
//    return dz.transpose() * invQj * dz + log(Qj.determinant());
}


/*
 * sensor_pose 雷达坐标
 * z 观测量
 * Q 传感器测量协方差
 * new_lm_pose 新地标坐标
 * new_lm_cov 新地标协方差
 */
void
FastSlam::AddNewLd(const Vec3d &sensor_pose, const Vec2d &z, const Mat2d &Q, Vec2d &new_lm_pose, Mat2d &new_lm_cov) {
    double angle_to_ld = normalize<double>(sensor_pose(2) + z(1));
    double s = sin(angle_to_ld);
    double c = cos(angle_to_ld);
    new_lm_pose << sensor_pose(0) + z(0) * c, sensor_pose(1) + z(0) * s;
    Mat2d Gz;
    Gz << c, -z(0) * s, s, z(0) * c;
    //H = h'(z,miu)
    //Cov = inv(H)*Q*inv(H)T
    //在这段代码中 Gz = h'(z,miu)-1 = inv(H)
    new_lm_cov = Gz * Q * Gz.transpose();
}

/*
 * sensor_pose 雷达坐标
 * sample 粒子
 * z 观测量
 * Q 传感器测量协方差
 * max_w 获得的最大权重值
 * max_i 能获得最大权重值的地标索引
 */
void
FastSlam::DataAssociate(const Vec3d &sensor_pose, const ParticleFilterSample &sample, const Vec2d &z, const Mat2d &Q,
                        double &max_w, int &max_i) {
    std::vector<double> weights;
    //Each landmarks int this particle, to obtain the max weight
    for (int j = 0; j < sample.landmark_num; j++) {
        Mat2d Hj;
        Mat2d Qj;
        Vec2d dz;
        ComputeJaccobians(sensor_pose, sample.lm_poses[j], sample.lm_covs[j],
                          z, Q, Hj, Qj, dz);
        weights.push_back(ComputeWeight(Qj, dz));
    }
    weights.push_back(new_ld_weight);

    std::cout << "weights:" << std::endl;

    max_w = -FLT_MIN;

    // obtain the max weight
    for (int wi = 0; wi < weights.size(); wi++) {
        std::cout << weights[wi] << " , ";
        if (weights[wi] > max_w) {
            max_w = weights[wi];
            max_i = wi;
        }
    }

    std::cout << std::endl;

    std::cout << "max index: " << max_i << " ,max weight: " << max_w << std::endl;

}


void FastSlam::UpdateKFwithCholesky(Vec2d &lm_pose, Mat2d &lm_cov, const Vec2d &dz, const Mat2d &Q, const Mat2d &Hj) {
    Mat2d PHt = lm_cov * Hj.transpose();
    Mat2d Qj = Hj * PHt + Q;//对应 Q=H*Covt-1*HT+Qt
    Qj = (Qj + Qj.transpose()) * 0.5;//make symmetric
    Mat2d QChol = Qj.llt().matrixL().transpose();
    Mat2d QjCholInv = QChol.inverse();
    Mat2d W1 = PHt * QjCholInv;
    Mat2d K = W1 * QjCholInv.transpose();
    lm_pose = lm_pose + K * dz;
    lm_cov = lm_cov - W1 * W1.transpose();
}
