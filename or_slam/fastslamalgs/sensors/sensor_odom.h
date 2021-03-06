#ifndef OR_FASTSLAM_ALG_SENSORS_SENSOR_ODOM_H
#define OR_FASTSLAM_ALG_SENSORS_SENSOR_ODOM_H

#include "memory"
#include <glog/logging.h>
#include "particle_filter/particle_filter.h"
#include "or_slam_math.h"

enum OdomModel {
    ODOM_MODEL_DIFF = 0,
};


class SensorOdomData {
public:
    Vec3d pose;
    Vec3d delta;
    Vec3d old_pose;
};

class SensorOdom {
public:

    SensorOdom(double alpha1,
               double alpha2,
               double alpha3,
               double alpha4);

    void SetModelDiff(double alpha1,
                      double alpha2,
                      double alpha3,
                      double alpha4);

    bool UpdateAction(std::shared_ptr<ParticleFilterSampleSet> set_ptr,
                      const SensorOdomData &odom_data);

private:

    double time_;
    OdomModel odom_model_type_;
    double alpha1_, alpha2_, alpha3_, alpha4_;
};


#endif
