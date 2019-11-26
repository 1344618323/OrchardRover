#ifndef OR_FASTSLAM_ALG_PARTICLE_FILTER_H
#define OR_FASTSLAM_ALG_PARTICLE_FILTER_H

#include <memory>
#include "log.h"
#include "particle_filter_sample.h"
#include "particle_filter_gaussian_pdf.h"

class ParticleFilter;

using ParticleFilterPtr = std::shared_ptr<ParticleFilter>;
using SampleSetPtr = std::shared_ptr<ParticleFilterSampleSet>;


class ParticleFilter {
public:
    ParticleFilter(int samples_num, const Vec3d &mean, const Mat3d &cov);

    ~ParticleFilter();

    void UpdateResample();

    SampleSetPtr GetCurrentSampleSetPtr() const {
        return this->sample_set_ptr_array_[current_set_];
    }

    SampleSetPtr GetNextSampleSetPtr() const {
        return this->sample_set_ptr_array_[(current_set_ + 1) % 2];
    }

    void ExchSetIndex() {
        current_set_ = (current_set_ + 1) % 2;
    }


private:
    void InitByGuassian(const Vec3d &mean, const Mat3d &cov);

private:
    int samples_num_;
    std::array<SampleSetPtr, 2> sample_set_ptr_array_;
    int current_set_;
};

#endif
