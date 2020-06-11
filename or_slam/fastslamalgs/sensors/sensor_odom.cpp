#include "sensor_odom.h"

// Only for odom model diff
SensorOdom::SensorOdom(double alpha1,
                       double alpha2,
                       double alpha3,
                       double alpha4) :
        alpha1_(alpha1),
        alpha2_(alpha2),
        alpha3_(alpha3),
        alpha4_(alpha4),
        time_(0.0) {
    odom_model_type_ = ODOM_MODEL_DIFF;
}

void SensorOdom::SetModelDiff(double alpha1, double alpha2, double alpha3, double alpha4) {
    odom_model_type_ = ODOM_MODEL_DIFF;
    alpha1_ = alpha1;
    alpha2_ = alpha2;
    alpha3_ = alpha3;
    alpha4_ = alpha4;
}

bool SensorOdom::UpdateAction(SampleSetPtr set_ptr, const SensorOdomData &odom_data) {

    // DLOG_INFO << "Compute the new sample poses by motion model";
    double delta_rot1, delta_trans, delta_rot2;
    double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
    double delta_rot1_noise, delta_rot2_noise;

    // Avoid computing a bearing from two poses that are extremely near each
    // other (happens on in-place rotation).
    if (sqrt(odom_data.delta[1] * odom_data.delta[1] +
             odom_data.delta[0] * odom_data.delta[0]) < 0.1)
        delta_rot1 = 0.0;
    else {
        delta_rot1 = AngleDiff<double>(atan2(odom_data.delta[1], odom_data.delta[0]), odom_data.old_pose[2]);
    }

    delta_trans = std::sqrt(odom_data.delta[0] * odom_data.delta[0] + odom_data.delta[1] * odom_data.delta[1]);
    delta_rot2 = AngleDiff<double>(odom_data.delta[2], delta_rot1);


    // We want to treat backward and forward motion symmetrically for the
    // noise model to be applied below.  The standard model seems to assume
    // forward motion.
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

    for (int i = 0; i < set_ptr->sample_count; i++) {

        delta_rot1_hat = AngleDiff<double>(delta_rot1, RandomGaussianNumByStdDev<double>(rot1_hat_stddev));
        delta_trans_hat = delta_trans - RandomGaussianNumByStdDev<double>(trans_hat_stddev);
        delta_rot2_hat = AngleDiff<double>(delta_rot2, RandomGaussianNumByStdDev<double>(rot2_hat_stddev));

        double delta_bearing = delta_rot1_hat + set_ptr->samples_vec[i].pose(2);
        double cs_bearing = std::cos(delta_bearing);
        double sn_bearing = std::sin(delta_bearing);

        set_ptr->samples_vec[i].pose[0] += (delta_trans_hat * cs_bearing);
        set_ptr->samples_vec[i].pose[1] += (delta_trans_hat * sn_bearing);
        set_ptr->samples_vec[i].pose[2] += delta_rot1_hat + delta_rot2_hat;
    }

    return true;
}


