//
// Created by cxn on 2020/7/6.
//

#ifndef OR_PLANNING_TEB_KINEMATICS_EDGE_H
#define OR_PLANNING_TEB_KINEMATICS_EDGE_H

#include <cmath>
#include "teb_vertex_pose.h"
#include "teb_penalties.h"
#include "teb_base_edge.h"

#define USE_ANALYTIC_JACOBI

namespace or_local_planner {
    /*
    差分车辆动力学约束边：车辆的位姿变化约束；避免后向移动的约束
    有雅克比矩阵
    */
    class KinematicsDiffDriveEdge : public TebBinaryEdgeBase<2, double, TebVertexPose, TebVertexPose> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        KinematicsDiffDriveEdge() {
            this->setMeasurement(0.);
        }

        void computeError() {
            const TebVertexPose *conf1 = static_cast<const TebVertexPose *>(_vertices[0]);
            const TebVertexPose *conf2 = static_cast<const TebVertexPose *>(_vertices[1]);

            Eigen::Vector2d deltaS = conf2->GetPose().GetPosition() - conf1->GetPose().GetPosition();

            /* 公式(5)
             * |cos(Bk)+cos(Bk+1)|  叉乘 dk,k+1           dy*(cos(Bk)+cos(Bk+1))-dx(sin(Bk)+sin(Bk+1)
             * |sin(Bk)+sin(Bk+1)|                =
             *
             * error=fabs(dy*(cos(Bk)+cos(Bk+1))-dx(sin(Bk)+sin(Bk+1))
             */
            _error[0] = fabs((cos(conf1->GetPose().GetTheta()) + cos(conf2->GetPose().GetTheta())) * deltaS[1]
                             - (sin(conf1->GetPose().GetTheta()) + sin(conf2->GetPose().GetTheta())) * deltaS[0]);

            //避免后向移动的约束：计算 dk,k+1 在 k坐标系下x的分量，若x>=0,则车前移，无惩罚；否则车后移，有惩罚
            Eigen::Vector2d angle_vec(cos(conf1->GetPose().GetTheta()), sin(conf1->GetPose().GetTheta()));
            _error[1] = PenaltyBoundFromBelow(deltaS.dot(angle_vec), 0, 0);
        }

#ifdef USE_ANALYTIC_JACOBI

        void linearizeOplus() {
            const TebVertexPose *conf1 = static_cast<const TebVertexPose *>(_vertices[0]);
            const TebVertexPose *conf2 = static_cast<const TebVertexPose *>(_vertices[1]);

            Eigen::Vector2d deltaS = conf2->GetPose().GetPosition() - conf1->GetPose().GetPosition();

            double cos1 = cos(conf1->GetPose().GetTheta());
            double cos2 = cos(conf2->GetPose().GetTheta());
            double sin1 = sin(conf1->GetPose().GetTheta());
            double sin2 = sin(conf2->GetPose().GetTheta());
            double aux1 = sin1 + sin2;
            double aux2 = cos1 + cos2;

            double dd_error_1 = deltaS[0] * cos1;
            double dd_error_2 = deltaS[1] * sin1;
            double dd_dev = PenaltyBoundFromBelowDerivative(dd_error_1 + dd_error_2, 0, 0);

            double dev_nh_abs = g2o::sign(
                    (cos(conf1->GetPose().GetTheta()) + cos(conf2->GetPose().GetTheta())) * deltaS[1] -
                    (sin(conf1->GetPose().GetTheta()) + sin(conf2->GetPose().GetTheta())) * deltaS[0]);

            _jacobianOplusXi(0, 0) = aux1 * dev_nh_abs;
            _jacobianOplusXi(0, 1) = -aux2 * dev_nh_abs;
            _jacobianOplusXi(1, 0) = -cos1 * dd_dev;
            _jacobianOplusXi(1, 1) = -sin1 * dd_dev;
            _jacobianOplusXi(0, 2) = (-dd_error_2 - dd_error_1) * dev_nh_abs;
            _jacobianOplusXi(1, 2) = (-sin1 * deltaS[0] + cos1 * deltaS[1]) * dd_dev;


            _jacobianOplusXj(0, 0) = -aux1 * dev_nh_abs;
            _jacobianOplusXj(0, 1) = aux2 * dev_nh_abs;
            _jacobianOplusXj(1, 0) = cos1 * dd_dev;
            _jacobianOplusXj(1, 1) = sin1 * dd_dev;
            _jacobianOplusXj(0, 2) = (-sin2 * deltaS[1] - cos2 * deltaS[0]) * dev_nh_abs;
            _jacobianOplusXj(1, 2) = 0;
        }

#endif
    };

    //非完整性车辆动力学约束边：车辆的位姿变化约束；转弯半径的约束
    class KinematicsCarlikeEdge : public TebBinaryEdgeBase<2, double, TebVertexPose, TebVertexPose> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        KinematicsCarlikeEdge() {
            this->setMeasurement(0.);
        }

        void computeError() {
            const TebVertexPose *conf1 = static_cast<const TebVertexPose *>(_vertices[0]);
            const TebVertexPose *conf2 = static_cast<const TebVertexPose *>(_vertices[1]);

            Eigen::Vector2d deltaS = conf2->GetPose().GetPosition() - conf1->GetPose().GetPosition();

            _error[0] = fabs((cos(conf1->GetPose().GetTheta()) + cos(conf2->GetPose().GetTheta())) * deltaS[1]
                             - (sin(conf1->GetPose().GetTheta()) + sin(conf2->GetPose().GetTheta())) * deltaS[0]);

            double angle_diff = g2o::normalize_theta(conf2->GetPose().GetTheta() - conf1->GetPose().GetTheta());
            if (angle_diff == 0) {
                _error[1] = 0;
            } else if (config_param_->trajectory_opt().exact_arc_length()) {
                //转弯半径必须要比最小转弯半径大
                //转弯半径 r= ||dk,k+1||/(2*sin(((Bk+1)-Bk)/2))，公式(6)就是该式的线性化版，原式更准但也更耗时
                _error[1] = PenaltyBoundFromBelow(fabs(deltaS.norm() / (2 * sin(angle_diff / 2))),
                                                  config_param_->kinematics_opt().min_turning_radius(), 0.0);
            } else {
                //论文中公式(6)： r=|vk/wk|≈||dk,k+1||/|(Bk+1)-Bk|
                _error[1] = PenaltyBoundFromBelow(deltaS.norm() / fabs(angle_diff),
                                                  config_param_->kinematics_opt().min_turning_radius(), 0.0);
            }
        }
    };
} // namespace or_local_planner
#endif //OR_PLANNING_TEB_KINEMATICS_EDGE_H