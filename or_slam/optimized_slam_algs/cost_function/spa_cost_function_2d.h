#ifndef OPTIMIZEDSLAM_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_2D_H_
#define OPTIMIZEDSLAM_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_2D_H_

#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/jet.h>
#include "common/math.h"
#include "cost_function/cost_helpers_impl.h"
#include "transform/rigid_transform.h"
#include "transform/transform.h"

namespace optimizedSlam {
    namespace optimization {

        struct PoseConstraint {
            optimizedSlam::transform::Rigid2d zbar_ij;
            double translation_weight;
            double rotation_weight;
        };

        struct LandmarkNode {
            struct LandmarkObservation {
//                common::Time time;
                int node_id;
                transform::Rigid2d landmark_to_tracking_transform;
//                double translation_weight;
//                double rotation_weight;
            };
            std::vector<LandmarkObservation> landmark_observations;
            transform::Rigid2d global_landmark_pose;
        };

        ceres::CostFunction *CreateAutoDiffSpaCostFunction(
                const optimizedSlam::optimization::PoseConstraint &pose);

        ceres::CostFunction *CreateAnalyticalSpaCostFunction(
                const optimizedSlam::optimization::PoseConstraint &pose);

    }  // namespace optimization
}  // namespace optimizedSlam

#endif  // OPTIMIZEDSLAM_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_2D_H_
