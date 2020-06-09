#include "fastslam.h"

FastSlam::FastSlam(const Vec3d &init_pose, const Vec3d &init_cov, ros::NodeHandle *nh, bool multi_sensor)
{
    multi_sensor_sign_ = multi_sensor;
    nh->param<int>("particles_num", particles_num_, 500);
    nh->param<double>("odom_alpha1", odom_alpha1_, 0.005);
    nh->param<double>("odom_alpha2", odom_alpha2_, 0.005);
    nh->param<double>("odom_alpha3", odom_alpha3_, 0.01);
    nh->param<double>("odom_alpha4", odom_alpha4_, 0.005);
    nh->param<double>("update_min_d", update_min_d_, 0.2); //在执行滤波更新前平移运动的距离0.05(50mm)与0.03（1.7度）
    nh->param<double>("update_min_a", update_min_a_, 0.5);

    observe_cov_.setZero();
    nh->param<double>("observe_cov_range", observe_cov_(0, 0), 0);
    nh->param<double>("observe_cov_bearing", observe_cov_(1, 1), 0);

    odom_model_ptr_ = std::make_unique<SensorOdom>(odom_alpha1_,
                                                   odom_alpha2_,
                                                   odom_alpha3_,
                                                   odom_alpha4_);

    init_pose_ = init_pose;
    init_cov_.setZero();
    init_cov_(0, 0) = init_cov(0);
    init_cov_(1, 1) = init_cov(1);
    init_cov_(2, 2) = init_cov(2);
    pf_ptr_ = std::make_shared<ParticleFilter>(particles_num_, init_pose_, init_cov_);
    pf_init_ = false;

    std::vector<std::string> csvtopic = {"error", "cnt"};
    csv_writer_ = std::make_unique<CsvWriter>("/home/cxn/error.csv", csvtopic);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            Vec2d p;
            p << 6.5 + j * 4, 6 + i * 4;
            true_lms_.push_back(p);
        }
    }

    //LOG_INFO << "FastSlam Alg Init!";
}

FastSlam::~FastSlam()
{
    //LOG_INFO << "FastSlam Alg Delete!";
}

int FastSlam::Update(const Vec3d &pose, const std::vector<Vec2d> &zs,
                     geometry_msgs::PoseArray &particle_cloud_pose_msg,
                     visualization_msgs::Marker &lm_cloud_msg)
{

    UpdateOdomPoseData(pose);

    if (zs.size() != 0 && odom_update_)
    {

        UpdateObserveData(zs);
        pf_ptr_->UpdateResample();
    }

    GetParticlesCloudMsg(particle_cloud_pose_msg, lm_cloud_msg);
    return 0;
}

void FastSlam::GetParticlesCloudMsg(geometry_msgs::PoseArray &particle_cloud_pose_msg,
                                    visualization_msgs::Marker &lm_cloud_msg)
{
    // particle_cloud_pose_msg.poses.resize(pf_ptr_->GetCurrentSampleSetPtr()->sample_count);

    // lm_cloud_msg.points.clear();

    /********20.1.3*******/
    double max_w = 0.0;
    int max_i = -1;

    for (int i = 0; i < pf_ptr_->GetCurrentSampleSetPtr()->sample_count; i++)
    {

        ParticleFilterSample &sample = pf_ptr_->GetCurrentSampleSetPtr()->samples_vec[i];
        // tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(sample.pose[2]),
        //                          tf::Vector3(sample.pose[0],
        //                                      sample.pose[1],
        //                                      0)),
        //                 particle_cloud_pose_msg.poses[i]);

        // for (int j = 0; j < sample.lm_vec.size(); j++)
        // {
        //     geometry_msgs::Point temp;
        //     temp.x = sample.lm_vec[j].pos(0);
        //     temp.y = sample.lm_vec[j].pos(1);
        //     temp.z = 0;
        //     lm_cloud_msg.points.push_back(temp);
        // }

        /********20.1.3*******/
        if (max_w < sample.weight)
        {
            max_w = sample.weight;
            max_i = i;
        }
    }

    /********20.1.3*******/
    gus_pose = pf_ptr_->GetCurrentSampleSetPtr()->samples_vec[max_i].pose;
    gus_lm_vec.clear();
    gus_lm_vec.shrink_to_fit();
    gus_lm_vec = pf_ptr_->GetCurrentSampleSetPtr()->samples_vec[max_i].lm_vec;

    particle_cloud_pose_msg.poses.resize(1);
    lm_cloud_msg.points.clear();
    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(gus_pose[2]),
                             tf::Vector3(gus_pose[0],
                                         gus_pose[1],
                                         0)),
                    particle_cloud_pose_msg.poses[0]);

    for (int j = 0; j < gus_lm_vec.size(); j++)
    {
        geometry_msgs::Point temp;
        temp.x = gus_lm_vec[j].pos(0);
        temp.y = gus_lm_vec[j].pos(1);
        temp.z = 0;
        lm_cloud_msg.points.push_back(temp);
    }

    if (gus_lm_vec.size() % 2 == 0 && gus_lm_vec.size() != 0)
    {
        double min = 0;
        for (int i = 0; i < gus_lm_vec.size(); i++)
        {
            min += mindistance(gus_lm_vec[i].pos);
        }
        csv_writer_->write(min/gus_lm_vec.size());
        csv_writer_->write(gus_lm_vec.size());
    }
}

void FastSlam::UpdateOdomPoseData(const Vec3d &pose)
{
    if (!pf_init_)
    {
        pf_odom_pose_ = pose;
        pf_init_ = true;
        odom_update_ = true;
    }
    else if (pf_init_)
    {
        Vec3d delta;
        // Compute change in pose
        delta[0] = pose[0] - pf_odom_pose_[0];
        delta[1] = pose[1] - pf_odom_pose_[1];
        delta[2] = AngleDiff<double>(pose[2], pf_odom_pose_[2]);

        // See if we should update the filter
        odom_update_ = std::fabs(delta[0]) > update_min_d_ ||
                       std::fabs(delta[1]) > update_min_d_ ||
                       std::fabs(delta[2]) > update_min_a_;

        if (odom_update_)
        {
            //DLOG_INFO << "Robot has moved, update the filter";
            SensorOdomData odom_data;
            odom_data.pose = pose;
            odom_data.delta = delta;
            odom_data.old_pose = pf_odom_pose_;
            odom_model_ptr_->UpdateAction(pf_ptr_->GetCurrentSampleSetPtr(), odom_data);
            pf_odom_pose_ = pose;
        }
    }
}

void FastSlam::SetSensorPose(const Vec3d &sensor_pose)
{
    sensor_pose_ = sensor_pose;
    //DLOG_INFO << "sensor pose: " << sensor_pose_[0] << ", " << sensor_pose_[1] << ", " << sensor_pose_[2];
}

void FastSlam::SetMultiSensorPose(const Vec3d &sensor_pose1, const Vec3d &sensor_pose2)
{
    multi_sensor_pose_[0] = sensor_pose1;
    multi_sensor_pose_[1] = sensor_pose2;
}

void FastSlam::UpdateObserveData(const std::vector<Vec2d> &zs)
{
    SampleSetPtr set_ptr = pf_ptr_->GetCurrentSampleSetPtr();

    std::cout << "!!!!!!!!!!! Update observe data: " << zs.size() << " !!!!!!!!!!!" << std::endl;

    for (int N = 0; N < zs.size(); N++)
    {

        // 如果测量范围不对,则跳过
        if (zs[N](0) <= 0.05 || zs[N](0) >= 8)
            continue;

        std::cout << "<<<<<<<<<< zs " << N << " <<<<<<<<<<" << std::endl;
        for (int k = 0; k < set_ptr->sample_count; k++)
        {
            ParticleFilterSample &sample = set_ptr->samples_vec[k];

            Vec3d sensor_pose;
            if (multi_sensor_sign_)
            {
                sensor_pose = CoordAdd(sample.pose, multi_sensor_pose_[N]);
            }
            else
            {
                //获得雷达坐标
                sensor_pose = CoordAdd(sample.pose, sensor_pose_);
            }

            std::cout << "sample index: " << k << std::endl;
            std::cout << "sensor pose: " << sensor_pose[0] << "," << sensor_pose[1] << ","
                      << sensor_pose[2] << ", sample pose: " << sample.pose[0] << "," << sample.pose[1] << ","
                      << sample.pose[2] << std::endl;

            double max_w;
            int max_i;

            DataAssociate(sensor_pose, sample, zs[N], observe_cov_, max_w, max_i);

            // update the weight of particle
            if (sample.weight * max_w > min_weight)
                sample.weight *= max_w;

            if (max_i == sample.lm_vec.size())
            {
                //new landmarks
                PfLandMark new_lm;
                AddNewLd(sensor_pose, zs[N], observe_cov_, new_lm);

                std::cout << "sample " << k << " add a new landmark: "
                          << new_lm.pos[0] << "," << new_lm.pos[1] << std::endl;

                sample.lm_vec.emplace_back(new_lm);
            }
            else if (max_i >= 0)
            {
                //update EKF
                Mat2d Hj;
                Mat2d Qj;
                Vec2d dz;
                ComputeJaccobians(sensor_pose, sample.lm_vec[max_i], zs[N], observe_cov_, Hj, Qj, dz);
                UpdateKfByCholesky(sample.lm_vec[max_i], dz, observe_cov_, Hj);
            }
        }
    }
}

/*
 * sensor_pose 雷达坐标
 * lm 地标坐标,协方差
 * z 观测量
 * Q 传感器测量协方差
 * Hj 观测函数对地标坐标的导数
 * Qj 观测协方差
 * dz 真实观测量与假设观测量之差
 */
void FastSlam::ComputeJaccobians(const Vec3d &sensor_pose, const PfLandMark &lm,
                                 const Vec2d &z, const Mat2d &Q,
                                 Mat2d &Hj, Mat2d &Qj, Vec2d &dz)
{
    // 粒子中地标与粒子坐标的差
    Vec2d dxy = lm.pos - sensor_pose.segment(0, 2);
    double d2 = dxy(0) * dxy(0) + dxy(1) * dxy(1);
    double d = sqrt(d2);

    Vec2d z_hat;
    z_hat(0) = d;
    z_hat(1) = AngleDiff<double>(atan2(dxy(1), dxy(0)), sensor_pose(2));

    //Hj 是观测函数h对地标位置(x,y)的偏导
    Hj << dxy(0) / d, dxy(1) / d,
        -dxy(1) / d2, dxy(0) / d2;

    Qj = Hj * lm.cov * Hj.transpose() + Q;
    dz << z(0) - z_hat(0), AngleDiff<double>(z(1), z_hat(1));
}

/*
 * Qj 观测协方差
 * dz 真实观测量与假设观测量之差
 * 输出权重
 */
double FastSlam::ComputeWeight(const Mat2d &Qj, const Vec2d &dz)
{
    bool invertible;
    Mat2d invQj;
    Qj.computeInverseWithCheck(invQj, invertible);
    if (!invertible)
    {
        return -1;
    }
    double mahalanobis_dis = dz.transpose() * invQj * dz;

    //    std::cout << ">>>>>>>>>>> " << mahalanobis_dis << " >>>>>>>>>>>" << std::endl;

    double weight = exp(-0.5 * mahalanobis_dis) / sqrt(Qj.determinant());

    return weight;
}

double FastSlam::ComputeMahalanobisDis(const Mat2d &Qj, const Vec2d &dz)
{
    bool invertible;
    Mat2d invQj;
    Qj.computeInverseWithCheck(invQj, invertible);
    if (!invertible)
    {
        return -1;
    }
    double mahalanobis_dis = dz.transpose() * invQj * dz;

    return mahalanobis_dis;
}

/*
 * sensor_pose 雷达坐标
 * z 观测量
 * Q 传感器测量协方差
 * new_lm_pose 新地标坐标
 * new_lm_cov 新地标协方差
 */
void FastSlam::AddNewLd(const Vec3d &sensor_pose,
                        const Vec2d &z, const Mat2d &Q,
                        PfLandMark &new_lm)
{
    double angle_to_ld = Normalize<double>(sensor_pose(2) + z(1));
    double s = sin(angle_to_ld);
    double c = cos(angle_to_ld);
    new_lm.pos << sensor_pose(0) + z(0) * c, sensor_pose(1) + z(0) * s;
    Mat2d Gz;
    Gz << c, -z(0) * s, s, z(0) * c;
    //H = h'(z,miu)
    //Cov = inv(H)*Q*inv(H)T
    //在这段代码中 Gz = h'(z,miu)-1 = inv(H)
    new_lm.cov = Gz * Q * Gz.transpose();
    new_lm.cnt = 1;
}

/*
 * sensor_pose 雷达坐标
 * sample 粒子
 * z 观测量
 * Q 传感器测量协方差
 * max_w 获得的最大权重值
 * max_i 能获得最大权重值的地标索引
 */
void FastSlam::DataAssociate(const Vec3d &sensor_pose, const ParticleFilterSample &sample,
                             const Vec2d &z, const Mat2d &Q,
                             double &max_w, int &max_i)
{
    std::vector<double> weights;
    std::vector<double> mah_dis;
    std::vector<Mat2d> Qjs;
    weights.resize(sample.lm_vec.size());
    mah_dis.resize(sample.lm_vec.size());
    Qjs.resize(sample.lm_vec.size());

    for (int j = 0; j < sample.lm_vec.size(); j++)
    {
        Mat2d Hj;
        Mat2d Qj;
        Vec2d dz;
        ComputeJaccobians(sensor_pose, sample.lm_vec[j],
                          z, Q, Hj, Qj, dz);
        // weights[j] = ComputeWeight(Qj, dz);

        mah_dis[j] = ComputeMahalanobisDis(Qj, dz); //若Qj不可逆,则返回-1
        Qjs[j] = Qj;
    }

    //    weights.push_back(new_ld_weight);
    //
    //    std::cout << "weights:" << std::endl;
    //
    //    max_w = -FLT_MIN;
    //
    //    // obtain the max weight
    //    for (int wi = 0; wi < weights.size(); wi++) {
    //        std::cout << weights[wi] << " , ";
    //        if (weights[wi] > max_w) {
    //            max_w = weights[wi];
    //            max_i = wi;
    //        }
    //    }

    //    std::cout << std::endl;
    //
    //    std::cout << "max index: " << max_i << " ,max weight: " << max_w << std::endl;

    double min_dis = FLT_MAX;
    int min_dis_i = -1;

    // obtain the max weight
    for (int i = 0; i < mah_dis.size(); i++)
    {
        std::cout << mah_dis[i] << " , ";
        if (mah_dis[i] < min_dis && mah_dis[i] >= 0)
        {
            min_dis = mah_dis[i];
            min_dis_i = i;
        }
    }

    if (min_dis >= 0 && min_dis < new_ld_mahalanobis_dis)
    {
        // 找到符合的地标
        max_i = min_dis_i;
        max_w = exp(-0.5 * min_dis) / sqrt(Qjs[min_dis_i].determinant());
    }
    else if (min_dis >= new_ld_mahalanobis_dis && min_dis_i >= 0)
    {
        // todo 应该再给min_dis设置一个下限?
        // 新地标
        max_i = mah_dis.size();
        max_w = 1;
    }

    if (min_dis_i == -1)
    {
        if (sample.lm_vec.size() == 0)
        {
            // 新地标
            max_i = mah_dis.size();
            max_w = 1;
        }
        else
        {
            // 已经有了多个地标,但是还是-1,就过掉这个
            max_i = -1;
            max_w = 1;
        }
    }

    std::cout << std::endl;
    std::cout << "max index: " << max_i << " , max weight: " << max_w << " , min dis: " << min_dis << std::endl;
}

void FastSlam::UpdateKfByCholesky(PfLandMark &lm,
                                  const Vec2d &dz, const Mat2d &Q, const Mat2d &Hj)
{
    Mat2d PHt = lm.cov * Hj.transpose();
    Mat2d Qj = Hj * PHt + Q;          //对应 Q=H*Covt-1*HT+Qt
    Qj = (Qj + Qj.transpose()) * 0.5; //make symmetric
    // Mat2d QChol = Qj.llt().matrixL().transpose();
    Mat2d QChol = Qj.llt().matrixU();

    //Mat2d QjCholInv = QChol.inverse();

    bool invertible;
    Mat2d QjCholInv;
    QChol.computeInverseWithCheck(QjCholInv, invertible);
    if (!invertible)
    {
        // 不可逆的话,直接结束
        return;
    }

    Mat2d W1 = PHt * QjCholInv;
    Mat2d K = W1 * QjCholInv.transpose();
    lm.pos = lm.pos + K * dz;
    lm.cov = lm.cov - W1 * W1.transpose();

    lm.cnt++;
}
