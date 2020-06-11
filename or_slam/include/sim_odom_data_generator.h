#ifndef OR_SLAM_SIM_SENSORS_SENSOR_ODOM_H
#define OR_SLAM_SIM_SENSORS_SENSOR_ODOM_H

#include <Eigen/Core>
#include "or_slam_math.h"

class SimOdomDataGenerator {
public:
    SimOdomDataGenerator(double alpha1,
                         double alpha2,
                         double alpha3,
                         double alpha4) {
        alpha1_ = alpha1;
        alpha2_ = alpha2;
        alpha3_ = alpha3;
        alpha4_ = alpha4;
        last_true_pose_.setZero();
        last_sim_pose_.setZero();
    }

    Eigen::Vector3d UpdateAction(const Eigen::Vector3d &odom_pose) {
        double delta_rot1, delta_trans, delta_rot2;
        double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
        double delta_rot1_noise, delta_rot2_noise;

        Eigen::Vector3d true_delta = odom_pose - last_true_pose_;
        if (sqrt(true_delta[1] * true_delta[1] + true_delta[0] * true_delta[0]) < 0.1)
            delta_rot1 = 0.0;
        else {
            delta_rot1 = AngleDiff<double>(atan2(true_delta[1], true_delta[0]), last_true_pose_[2]);
        }
        delta_trans = std::sqrt(true_delta[0] * true_delta[0] + true_delta[1] * true_delta[1]);
        delta_rot2 = AngleDiff<double>(true_delta[2], delta_rot1);

        delta_rot1_noise = std::min(fabs(AngleDiff<double>(delta_rot1, 0.0)),
                                    fabs(AngleDiff<double>(delta_rot1, M_PI)));
        delta_rot2_noise = std::min(fabs(AngleDiff<double>(delta_rot2, 0.0)),
                                    fabs(AngleDiff<double>(delta_rot2, M_PI)));

        double rot1_hat_stddev = std::sqrt(
                alpha1_ * delta_rot1_noise * delta_rot1_noise +
                alpha2_ * delta_trans * delta_trans);
        double trans_hat_stddev = std::sqrt(
                alpha3_ * delta_trans * delta_trans +
                alpha4_ * delta_rot1_noise * delta_rot1_noise +
                alpha4_ * delta_rot2_noise * delta_rot2_noise);
        double rot2_hat_stddev = std::sqrt(
                alpha1_ * delta_rot2_noise * delta_rot2_noise +
                alpha2_ * delta_trans * delta_trans);

        delta_rot1_hat = AngleDiff<double>(delta_rot1, RandomGaussianNumByStdDev<double>(rot1_hat_stddev));
        delta_trans_hat = delta_trans - RandomGaussianNumByStdDev<double>(trans_hat_stddev);
        delta_rot2_hat = AngleDiff<double>(delta_rot2, RandomGaussianNumByStdDev<double>(rot2_hat_stddev));

        double delta_bearing = delta_rot1_hat + last_sim_pose_(2);
        double cs_bearing = std::cos(delta_bearing);
        double sn_bearing = std::sin(delta_bearing);

        Eigen::Vector3d sim_pose;

        sim_pose[0] = last_sim_pose_[0] + (delta_trans_hat * cs_bearing);
        sim_pose[1] = last_sim_pose_[1] + (delta_trans_hat * sn_bearing);
        sim_pose[2] = last_sim_pose_[2] + delta_rot1_hat + delta_rot2_hat;

        last_sim_pose_ = sim_pose;
        last_true_pose_ = odom_pose;
        return sim_pose;
    }


    Eigen::Vector3d UpdateAction2(const Eigen::Vector3d &odom_pose) {
        double delta_rot, delta_trans_x, delta_trans_y;

        Eigen::Vector3d true_delta = odom_pose - last_true_pose_;

        if (fabs(true_delta.x()) < 0.1 && fabs(true_delta.y()) < 0.1 && fabs(true_delta.z()) < 0.1) {
            return last_sim_pose_;
        }

        delta_rot = RandomGaussianNumByStdDev<double>(alpha1_);
        delta_trans_x = RandomGaussianNumByStdDev<double>(alpha2_);
        delta_trans_y = RandomGaussianNumByStdDev<double>(alpha3_);

        Eigen::Vector3d sim_pose;

        sim_pose[0] = last_sim_pose_[0] + true_delta[0] + delta_trans_x;
        sim_pose[1] = last_sim_pose_[1] + true_delta[1] + delta_trans_y;
        sim_pose[2] = last_sim_pose_[2] + true_delta[2] + delta_rot;
        sim_pose[2] = Normalize<double>(sim_pose[2]);

        last_sim_pose_ = sim_pose;
        last_true_pose_ = odom_pose;
        return sim_pose;
    }


private:
    double alpha1_, alpha2_, alpha3_, alpha4_;
    Eigen::Vector3d last_true_pose_;
    Eigen::Vector3d last_sim_pose_;
};

#endif
