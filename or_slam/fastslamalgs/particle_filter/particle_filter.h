#ifndef OR_FASTSLAM_ALG_PARTICLE_FILTER_H
#define OR_FASTSLAM_ALG_PARTICLE_FILTER_H

#include <memory>
#include <glog/logging.h>
#include "particle_filter_sample.h"
#include "particle_filter_gaussian_pdf.h"

class ParticleFilter;

using ParticleFilterPtr = std::shared_ptr<ParticleFilter>;
using SampleSetPtr = std::shared_ptr<ParticleFilterSampleSet>;

class ParticleFilter
{
public:
    ParticleFilter(int samples_num, const Vec3d &mean, const Mat3d &cov);

    ~ParticleFilter();

    bool UpdateResample();

    SampleSetPtr GetCurrentSampleSetPtr() const
    {
        return this->sample_set_ptr_array_[current_set_];
    }

private:
    void InitByGuassian(const Vec3d &mean, const Mat3d &cov);

    double NormalizeWeight();

private:
    int samples_num_;

    double Neff_threshold_;

    std::array<SampleSetPtr, 2> sample_set_ptr_array_;
    int current_set_;
};

#endif
