#ifndef OR_FASTSLAM_ALG_PARTICLE_FILTER_SAMPLE_H
#define OR_FASTSLAM_ALG_PARTICLE_FILTER_SAMPLE_H
#include "types.h"


class PfLandMark{
public:
    Vec2d pos;
    Mat2d cov;
    int cnt;
};

/**
* @brief Information for a single sample.
*/
class ParticleFilterSample {
public:
    ParticleFilterSample() {
        Reset();
    }

    void Reset() {
        pose.setZero();
        weight = 0;
        lm_vec.clear();
        lm_vec.shrink_to_fit();
    };

    ~ParticleFilterSample(){
        lm_vec.clear();
        lm_vec.shrink_to_fit();
    }

public:
    //! Pose represented by this sample
    Vec3d pose;

    //! Weight for this pose
    double weight;

    std::vector<PfLandMark> lm_vec;
};


/**
* @brief Information for a set of samples
*/
class ParticleFilterSampleSet {
public:
    //! Number of samples
    int sample_count = 0;
    //! Vector of samples
    std::vector<ParticleFilterSample> samples_vec;
    //! Filter statistics mean
    Vec3d mean;
    //! Filter statistics covariant
    Mat3d covariant;
};

#endif //PROJECT_PARTICLE_FILTER_SAMPLE_H
