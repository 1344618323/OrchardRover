#include "particle_filter.h"

ParticleFilter::ParticleFilter(int samples_num, const Vec3d &mean, const Mat3d &cov) {
    samples_num_ = samples_num;
    current_set_ = 0;
    for (auto &sample_set_it : sample_set_ptr_array_) {

        sample_set_it = std::make_shared<ParticleFilterSampleSet>();

        sample_set_it->sample_count = samples_num_;
        sample_set_it->samples_vec.resize(samples_num_);
        for (int i = 0; i < samples_num_; i++) {
            sample_set_it->samples_vec[i].weight = 1.0 / samples_num_;
        }
        sample_set_it->mean.setZero();
        sample_set_it->covariant.setZero();
    }

    InitByGuassian(mean, cov);
    LOG_INFO << "ParticleFilter Init!!";
}


ParticleFilter::~ParticleFilter() {
    for (int i = 0; i < 2; i++) {
        sample_set_ptr_array_[i]->samples_vec.clear();
        sample_set_ptr_array_[i]->samples_vec.shrink_to_fit();
    }
    LOG_INFO << "Delete pf!";
}

void ParticleFilter::InitByGuassian(const Vec3d &mean, const Mat3d &cov) {

    auto pf_gaussian_pdf = ParticleFilterGaussianPdf(mean, cov);

    // Compute the new sample poses
    for (int i = 0; i < sample_set_ptr_array_[current_set_]->sample_count; i++) {
        sample_set_ptr_array_[current_set_]->samples_vec[i].weight = 1.0 / samples_num_;
        sample_set_ptr_array_[current_set_]->samples_vec[i].pose = pf_gaussian_pdf.GenerateSample();
    }
}

void ParticleFilter::NormalizeWeight() {
    double sum = 0;
    SampleSetPtr set_ptr = GetCurrentSampleSetPtr();
    for (int k = 0; k < set_ptr->sample_count; k++) {
        sum += set_ptr->samples_vec[k].weight;
    }
    for (int k = 0; k < set_ptr->sample_count; k++) {
        set_ptr->samples_vec[k].weight /= sum;
    }
}

void ParticleFilter::UpdateResample() {
    SampleSetPtr set_ptr = GetCurrentSampleSetPtr();
    SampleSetPtr next_set_ptr = sample_set_ptr_array_[(current_set_ + 1) % 2];
    NormalizeWeight();
    int sample_count = set_ptr->sample_count;
    double pw[sample_count];
    double sample_base[sample_count];
    pw[0] = set_ptr->samples_vec[0].weight;
    sample_base[0] = drand48() / sample_count;
    for (int k = 1; k < sample_count; k++) {
        pw[k] = pw[k - 1] + set_ptr->samples_vec[k].weight;
        sample_base[k] = sample_base[k - 1] + 1.0 / sample_count;
    }
    int ind = 0;
    for (int k = 0; k < sample_count; k++) {
        while (ind < set_ptr->sample_count && sample_base[k] > pw[ind]) {
            ind++;
        }
        next_set_ptr->samples_vec[k] = set_ptr->samples_vec[ind];
        next_set_ptr->samples_vec[k].weight = 1.0 / sample_count;
    }
    current_set_ = (current_set_ + 1) % 2;
}


