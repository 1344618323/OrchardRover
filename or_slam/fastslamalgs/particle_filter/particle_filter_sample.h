#ifndef OR_FASTSLAM_ALG_PARTICLE_FILTER_SAMPLE_H
#define OR_FASTSLAM_ALG_PARTICLE_FILTER_SAMPLE_H
#include "types.h"

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
        lm_poses.clear();
        lm_poses.shrink_to_fit();
        lm_covs.clear();
        lm_covs.shrink_to_fit();
        lm_cnt.clear();
        lm_cnt.shrink_to_fit();
        landmark_num=0;
    };

    ~ParticleFilterSample(){
        lm_poses.clear();
        lm_poses.shrink_to_fit();
        lm_covs.clear();
        lm_covs.shrink_to_fit();
        lm_cnt.clear();
        lm_cnt.shrink_to_fit();
    }

public:
    //! Pose represented by this sample
    Vec3d pose;

    //! Weight for this pose
    double weight;

    std::vector<Vec2d> lm_poses;
    std::vector<Mat2d> lm_covs;
    std::vector<int> lm_cnt;
    int landmark_num;
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
