#include "particle_filter.h"

ParticleFilter::ParticleFilter(int samples_num, const Vec3d &mean, const Mat3d &cov) {
    samples_num_ = samples_num;
    current_set_ = 0;
    for (int i = 0; i < 2; i++) {
        sample_set_ptr_array_[i] = std::make_shared<ParticleFilterSampleSet>();

        sample_set_ptr_array_[i]->sample_count = samples_num_;
        sample_set_ptr_array_[i]->samples_vec.resize(samples_num_);
        for (int i = 0; i < sample_set_ptr_array_[i]->sample_count; i++) {
            sample_set_ptr_array_[i]->samples_vec[i].weight = 1.0 / samples_num_;
        }
        sample_set_ptr_array_[i]->mean.setZero();
        sample_set_ptr_array_[i]->covariant.setZero();
    }
    InitByGuassian(mean, cov);
}


ParticleFilter::~ParticleFilter() {
    for (int i = 0; i < 2; i++) {
        sample_set_ptr_array_[i]->samples_vec.clear();
        sample_set_ptr_array_[i]->samples_vec.shrink_to_fit();
    }
    LOG_INFO << "Delete pf";
}


void ParticleFilter::UpdateResample() {

//    double w_diff = 0;
//    double total = 0;
//    int i = 0;
//
//    ParticleFilterSample *sample_a, *sample_b;
//
//    auto set_a = this->sample_set_ptr_array_[current_set_];
//    auto set_b = this->sample_set_ptr_array_[(current_set_ + 1) % 2];
//
//    // Build up cumulative probability table for resampling.
//    // TODO: Replace this with a more efficient procedure
//    // (e.g., http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
//    const int c_size = set_a->sample_count + 1;
//    std::vector<double> c;
//    c.resize(c_size);
//    c[0] = 0.0;
//    for (i = 0; i < set_a->sample_count; i++) {
//        c[i + 1] = c[i] + set_a->samples_vec[i].weight;
//    }
//
//    set_b->kd_tree_ptr->Clear();
//
//    // Draw samples from set a to create set b.
//    total = 0;
//    set_b->sample_count = 0;
//
//    w_diff = 1.0 - this->w_fast_ / this->w_slow_;
//    if (w_diff < 0.0) {
//        w_diff = 0.0;
//    }
//
//    while (set_b->sample_count < this->max_samples_) {
//        sample_b = &set_b->samples_vec[set_b->sample_count++];
//
//        if (drand48() < w_diff)
//            sample_b->pose = (random_pose_func_)();
//        else {
//            double r;
//            r = drand48();
//
//            //好像是转盘采样
//            for (i = 0; i < set_a->sample_count; i++) {
//                if ((c[i] <= r) && (r < c[i + 1]))
//                    break;
//            }
//            assert(i < set_a->sample_count);
//            sample_a = &(set_a->samples_vec[i]);
//
//            assert(sample_a->weight > 0);
//            sample_b->pose = sample_a->pose;
//        }
//
//        sample_b->weight = 1.0;
//        total += sample_b->weight;
//
//        // Add sample to histogram
//        set_b->kd_tree_ptr->InsertPose(sample_b->pose, sample_b->weight);
//
//        // See if we have enough samples yet
//        DLOG_INFO << "Histogram bins num: " << set_b->kd_tree_ptr->GetLeafCount();
//        auto kld_resample_num = ResampleLimit(set_b->kd_tree_ptr->GetLeafCount());
//        if (set_b->sample_count > kld_resample_num) {
//            LOG_INFO << "KLD-Resample num : " << kld_resample_num;
//            break;
//        }
//    }
//
//    // Reset averages, to avoid spiraling off into complete randomness.
//    // 为了便面一直生成随机粒子
//    if (w_diff > 0.0)
//        this->w_slow_ = this->w_fast_ = 0.0;
//
//    // Normalize weights
//    for (i = 0; i < set_b->sample_count; i++) {
//        sample_b = &(set_b->samples_vec[i]);
//        sample_b->weight /= total;
//    }
}


void ParticleFilter::InitByGuassian(const Vec3d &mean, const Mat3d &cov) {

    auto pf_gaussian_pdf = ParticleFilterGaussianPdf(mean, cov);

    // Compute the new sample poses
    for (int i = 0; i < sample_set_ptr_array_[current_set_]->sample_count; i++) {
        sample_set_ptr_array_[current_set_]->samples_vec[i].weight = 1.0 / samples_num_;
        sample_set_ptr_array_[current_set_]->samples_vec[i].pose = pf_gaussian_pdf.GenerateSample();
    }
}



