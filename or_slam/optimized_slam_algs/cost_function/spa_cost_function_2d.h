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
                //Time time;
                int node_id;
                Eigen::Vector2d landmark_to_tracking_transform;
            };
            std::vector<LandmarkObservation> landmark_observations;
            Eigen::Vector2d global_landmark_xy;
            int visible;
            std::vector<int> latest_obs_node_id;
            LandmarkNode() {
                visible = 0;
            }
        };


        class SpaCostFunction2D {
        public:
            explicit SpaCostFunction2D(
                    const optimizedSlam::optimization::PoseConstraint &observed_relative_pose);

            template<typename T>
            bool operator()(const T *const start_pose, const T *const end_pose, T *e) const;

        private:
            const optimizedSlam::optimization::PoseConstraint observed_relative_pose_;
        };

        class AnalyticalSpaCostFunction2D
                : public ceres::SizedCostFunction<3 /* number of residuals */,
                        3 /* size of start pose */,
                        3 /* size of end pose */> {
        public:
            explicit AnalyticalSpaCostFunction2D(
                    const optimizedSlam::optimization::PoseConstraint &constraint_pose);

            virtual ~AnalyticalSpaCostFunction2D();

            bool Evaluate(double const *const *parameters, double *residuals,
                          double **jacobians) const override;

        private:
            const transform::Rigid2d observed_relative_pose_;
            const double translation_weight_;
            const double rotation_weight_;
        };

        ceres::CostFunction *CreateAutoDiffSpaCostFunction(
                const optimizedSlam::optimization::PoseConstraint &observed_relative_pose);

        ceres::CostFunction *CreateAnalyticalSpaCostFunction(
                const optimizedSlam::optimization::PoseConstraint &observed_relative_pose);

        struct PoselmConstraint {
            Eigen::Vector2d zbar_ij;
            double translation_weight;
        };


        class LmCostFunction2D {
        public:
            explicit LmCostFunction2D(
                    const optimizedSlam::optimization::PoselmConstraint &observed_relative_xy);

            template<typename T>
            bool operator()(const T *const node_pose, const T *const lm_xy,
                            T *e) const;

        private:
            const optimizedSlam::optimization::PoselmConstraint observed_relative_xy_;
        };

        ceres::CostFunction *CreateAutoDiffLmCostFunction(
                const optimizedSlam::optimization::PoselmConstraint &observed_relative_xy);

    } // namespace optimization
} // namespace optimizedSlam

#endif // OPTIMIZEDSLAM_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_2D_H_
