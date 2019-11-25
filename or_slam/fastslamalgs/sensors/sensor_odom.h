#ifndef OR_FASTSLAM_ALG_SENSORS_SENSOR_ODOM_H
#define OR_FASTSLAM_ALG_SENSORS_SENSOR_ODOM_H

#include "memory"
#include "log.h"
#include "particle_filter/particle_filter_sample.h"
#include "fastslam_math.h"

enum OdomModel{
  ODOM_MODEL_DIFF = 0,
  ODOM_MODEL_OMNI = 1
};


class SensorOdomData{
 public:
  Vec3d pose;
  Vec3d delta;
};

class SensorOdom {
 public:
  /**
   * @brief Default constructor
   */
  SensorOdom(double alpha1,
             double alpha2,
             double alpha3,
             double alpha4);

  /**
   * @brief Set odometry model to omni model.
   * @param alpha1
   * @param alpha2
   * @param alpha3
   * @param alpha4
   */
  void SetModelDiff(double alpha1,
                    double alpha2,
                    double alpha3,
                    double alpha4);

  /**
   * @brief Update the filter based on the action model
   * @param pf_ptr Particle filter object pointer
   * @param sensor_data_ptr Sensor data object pointer
   * @return Returns true if the filter has been updated
   */
  bool UpdateAction(std::shared_ptr<ParticleFilterSampleSet> pf_sample_set_ptr,
                    const SensorOdomData &odom_data);

 private:

  /**
   * @brief Current data timestamp
   */
  double time_;

  /**
   * @brief Model type
   */
  OdomModel odom_model_type_;

  /**
   * @brief Drift parameters
   */
  double alpha1_, alpha2_, alpha3_, alpha4_;
};


#endif
