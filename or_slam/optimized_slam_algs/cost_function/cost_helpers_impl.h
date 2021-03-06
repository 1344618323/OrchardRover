#ifndef OPTIMIZEDSLAM_OPTIMIZATION_COST_FUNCTIONS_COST_HELPERS_IMPL_H_
#define OPTIMIZEDSLAM_OPTIMIZATION_COST_FUNCTIONS_COST_HELPERS_IMPL_H_

#include <Eigen/Core>
#include "transform/rigid_transform.h"
#include "transform/transform.h"

namespace optimized_slam {
    namespace optimization {

/*(cxn)?
  计算误差的公式不是 T_{ij}^{m}'*T_{i}'*T_{j}，而是 https://google-cartographer.readthedocs.io/en/latest/cost_functions.html
  
  而是 T_{ij}^{m} - (T_{i}^{-1} * T_{j}) （2d中还好解释，3d中的旋转是什么鬼）
  T_{i}^{-1} * T_{j} = [ R_{i}^{-1} * R_{j} ; R_{i}^{-1} * ( t_{j} - t_{i} ) ]

  在2d中，err = [ t_{ij}^{m} - R_{i}^{-1} * ( t_{j} - t_{i} )  ;  theta_{ij} - (theta_{j}-theta_{i})]

  在3d中，err = [ t_{ij}^{m} - R_{i}^{-1} * ( t_{j} - t_{i} )  ;  ( R_{i}^{-1} * R_{j} )^{-1} * R_{ij}^{m} ]
  注意第二项在矩阵乘法结束后，要转成旋转向量
  
  所以说这两个公式咋来的？还是因人而异，没有固定的方式？
*/

        template<typename T>
        static std::array<T, 3> ComputeUnscaledError(
                const transform::Rigid2d &relative_pose, const T *const start,
                const T *const end) {
            const T cos_theta_i = cos(start[2]);
            const T sin_theta_i = sin(start[2]);
            const T delta_x = end[0] - start[0];
            const T delta_y = end[1] - start[1];
            const T h[3] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                            -sin_theta_i * delta_x + cos_theta_i * delta_y,
                            end[2] - start[2]};
            return {{T(relative_pose.translation().x()) - h[0],
                            T(relative_pose.translation().y()) - h[1],
                            common::NormalizeAngleDifference(
                                    T(relative_pose.rotation().angle()) - h[2])}};
        }

        template<typename T>
        std::array<T, 3> ScaleError(const std::array<T, 3> &error,
                                    double translation_weight, double rotation_weight) {
            // clang-format off
            return {{
                            error[0] * translation_weight,
                            error[1] * translation_weight,
                            error[2] * rotation_weight
                    }};
            // clang-format on
        }

        template<typename T>
        static std::array<T, 2> ComputeUnscaledError2D(
                const Eigen::Vector2d &relative_pose, const T *const node_pose,
                const T *const lm_xy) {
            const T cos_theta_i = cos(node_pose[2]);
            const T sin_theta_i = sin(node_pose[2]);
            const T delta_x = lm_xy[0] - node_pose[0];
            const T delta_y = lm_xy[1] - node_pose[1];
            const T h[2] = {cos_theta_i * delta_x + sin_theta_i * delta_y,
                            -sin_theta_i * delta_x + cos_theta_i * delta_y};
            return {{T(relative_pose.x()) - h[0],
                            T(relative_pose.y()) - h[1]}};
        }

        template<typename T>
        std::array<T, 2> ScaleError2D(const std::array<T, 2> &error,
                                    double translation_weight) {
            // clang-format off
            return {{
                            error[0] * translation_weight,
                            error[1] * translation_weight,
                    }};
            // clang-format on
        }




        template<typename T>
        static std::array<T, 6> ComputeUnscaledError(
                const transform::Rigid3d &relative_pose, const T *const start_rotation,
                const T *const start_translation, const T *const end_rotation,
                const T *const end_translation) {
            const Eigen::Quaternion<T> R_i_inverse(start_rotation[0], -start_rotation[1],
                                                   -start_rotation[2],
                                                   -start_rotation[3]);

            const Eigen::Matrix<T, 3, 1> delta(end_translation[0] - start_translation[0],
                                               end_translation[1] - start_translation[1],
                                               end_translation[2] - start_translation[2]);
            const Eigen::Matrix<T, 3, 1> h_translation = R_i_inverse * delta;

            const Eigen::Quaternion<T> h_rotation_inverse =
                    Eigen::Quaternion<T>(end_rotation[0], -end_rotation[1], -end_rotation[2],
                                         -end_rotation[3]) *
                    Eigen::Quaternion<T>(start_rotation[0], start_rotation[1],
                                         start_rotation[2], start_rotation[3]);

            const Eigen::Matrix<T, 3, 1> angle_axis_difference =
                    transform::RotationQuaternionToAngleAxisVector(
                            h_rotation_inverse * relative_pose.rotation().cast<T>());

            return {{T(relative_pose.translation().x()) - h_translation[0],
                            T(relative_pose.translation().y()) - h_translation[1],
                            T(relative_pose.translation().z()) - h_translation[2],
                            angle_axis_difference[0], angle_axis_difference[1],
                            angle_axis_difference[2]}};
        }

        template<typename T>
        std::array<T, 6> ScaleError(const std::array<T, 6> &error,
                                    double translation_weight, double rotation_weight) {
            // clang-format off
            return {{
                            error[0] * translation_weight,
                            error[1] * translation_weight,
                            error[2] * translation_weight,
                            error[3] * rotation_weight,
                            error[4] * rotation_weight,
                            error[5] * rotation_weight
                    }};
            // clang-format on
        }

//  Eigen implementation of slerp is not compatible with Ceres on all supported
//  platforms. Our own implementation is used instead.
        template<typename T>
        std::array<T, 4> SlerpQuaternions(const T *const start, const T *const end,
                                          double factor) {
            // Angle 'theta' is the half-angle "between" quaternions. It can be computed
            // as the arccosine of their dot product.
            const T cos_theta = start[0] * end[0] + start[1] * end[1] +
                                start[2] * end[2] + start[3] * end[3];
            // Avoid using ::abs which would cast to integer.
            const T abs_cos_theta = ceres::abs(cos_theta);
            // If numerical error brings 'cos_theta' outside [-1 + epsilon, 1 - epsilon]
            // interval, then the quaternions are likely to be collinear.
            T prev_scale(1. - factor);
            T next_scale(factor);
            if (abs_cos_theta < T(1. - 1e-5)) {
                const T theta = acos(abs_cos_theta);
                const T sin_theta = sin(theta);
                prev_scale = sin((1. - factor) * theta) / sin_theta;
                next_scale = sin(factor * theta) / sin_theta;
            }
            if (cos_theta < T(0.)) {
                next_scale = -next_scale;
            }
            return {{prev_scale * start[0] + next_scale * end[0],
                            prev_scale * start[1] + next_scale * end[1],
                            prev_scale * start[2] + next_scale * end[2],
                            prev_scale * start[3] + next_scale * end[3]}};
        }

// 简单的线性插值
        template<typename T>
        std::tuple<std::array<T, 4> /* rotation */, std::array<T, 3> /* translation */>
        InterpolateNodes3D(const T *const prev_node_rotation,
                           const T *const prev_node_translation,
                           const T *const next_node_rotation,
                           const T *const next_node_translation,
                           const double interpolation_parameter) {
            return std::make_tuple(
                    SlerpQuaternions(prev_node_rotation, next_node_rotation,
                                     interpolation_parameter),
                    std::array<T, 3>{
                            {prev_node_translation[0] +
                             interpolation_parameter *
                             (next_node_translation[0] - prev_node_translation[0]),
                                    prev_node_translation[1] +
                                    interpolation_parameter *
                                    (next_node_translation[1] - prev_node_translation[1]),
                                    prev_node_translation[2] +
                                    interpolation_parameter *
                                    (next_node_translation[2] - prev_node_translation[2])}});
        }

        template<typename T>
        std::tuple<std::array<T, 4> /* rotation */, std::array<T, 3> /* translation */>
        InterpolateNodes2D(const T *const prev_node_pose,
                           const Eigen::Quaterniond &prev_node_gravity_alignment,
                           const T *const next_node_pose,
                           const Eigen::Quaterniond &next_node_gravity_alignment,
                           const double interpolation_parameter) {
            // The following is equivalent to (Embed3D(prev_node_pose) *
            // Rigid3d::Rotation(prev_node_gravity_alignment)).rotation().
            const Eigen::Quaternion<T> prev_quaternion(
                    (Eigen::AngleAxis<T>(prev_node_pose[2], Eigen::Matrix<T, 3, 1>::UnitZ()) *
                     prev_node_gravity_alignment.cast<T>())
                            .normalized());
            const std::array<T, 4> prev_node_rotation = {
                    {prev_quaternion.w(), prev_quaternion.x(), prev_quaternion.y(),
                            prev_quaternion.z()}};

            // The following is equivalent to (Embed3D(next_node_pose) *
            // Rigid3d::Rotation(next_node_gravity_alignment)).rotation().
            const Eigen::Quaternion<T> next_quaternion(
                    (Eigen::AngleAxis<T>(next_node_pose[2], Eigen::Matrix<T, 3, 1>::UnitZ()) *
                     next_node_gravity_alignment.cast<T>())
                            .normalized());
            const std::array<T, 4> next_node_rotation = {
                    {next_quaternion.w(), next_quaternion.x(), next_quaternion.y(),
                            next_quaternion.z()}};

            return std::make_tuple(
                    SlerpQuaternions(prev_node_rotation.data(), next_node_rotation.data(),
                                     interpolation_parameter),
                    std::array<T, 3>{
                            {prev_node_pose[0] + interpolation_parameter *
                                                 (next_node_pose[0] - prev_node_pose[0]),
                                    prev_node_pose[1] + interpolation_parameter *
                                                        (next_node_pose[1] - prev_node_pose[1]),
                                    T(0)}});
        }

    }  // namespace optimization
}  // namespace optimized_slam

#endif  // OPTIMIZEDSLAM_OPTIMIZATION_COST_FUNCTIONS_COST_HELPERS_IMPL_H_
