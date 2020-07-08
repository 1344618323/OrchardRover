//
// Created by cxn on 2020/7/6.
//

#ifndef OR_PLANNING_TEB_VELOCITY_EDGE_H
#define OR_PLANNING_TEB_VELOCITY_EDGE_H

#include "teb_vertex_pose.h"
#include "teb_vertex_timediff.h"
#include "teb_base_edge.h"
#include "teb_penalties.h"
#include "utility_tool.h"

namespace or_local_planner {

    //非完整性约束速度边
    class VelocityEdge : public TebMultiEdgeBase<2, double> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //三元边
        VelocityEdge() {
            this->resize(3);
        }

        void computeError() {
            const TebVertexPose *conf1 = static_cast<const TebVertexPose *>(_vertices[0]);
            const TebVertexPose *conf2 = static_cast<const TebVertexPose *>(_vertices[1]);
            const TebVertexTimeDiff *deltaT = static_cast<const TebVertexTimeDiff *>(_vertices[2]);

            const Eigen::Vector2d deltaS = conf2->estimate().GetPosition() - conf1->estimate().GetPosition();

            double dist = deltaS.norm();
            const double angle_diff = g2o::normalize_theta(conf2->GetPose().GetTheta() - conf1->GetPose().GetTheta());
            if (config_param_->trajectory_opt().exact_arc_length() && angle_diff != 0) {
                // 如果为真，则规划器在速度、加速度和转弯率计算中使用精确的弧长（->增加的cpu时间），否则使用欧几里德近似。
                // 根据首尾位姿角度差算出 车辆行驶过程中的转弯半径radius
                // angle_diff就是 车辆绕radius的圆的 转弯弧度，
                // radius= (dist/2)/(sin(angle_diff/2)) -> 弧长公式算出行驶距离 dist=|angle_diff*radius|
                double radius = dist / (2 * sin(angle_diff / 2));
                dist = fabs(angle_diff * radius);
            }
            double vel = dist / deltaT->estimate();


            // 论文公式(7):vk=dist/delta(T)× y(sk,sk+1)
            // y(sk,sk+1)用来判断vk的正负号（dist一定是>=0）
            // 公式(9): y(sk,sk+1) = sign(<qk dk,k+1>)
            // 我们知道 Twk=[cos(Bk) -sin(Bk); sin(Bk) cos(Bk)] ，而Tkw=[cos(Bk) sin(Bk); -sin(Bk) cos(Bk)]
            // 因此可算出在k坐标系下 dk,k+1的x分量为为 qk 与 dk,k+1的内积，也就是<qk dk,k+1>，
            // 若 sign(<qk dk,k+1>)=1，则车往前开，vk为正；若 sign(<qk dk,k+1>)=-1，则车往后开，vk为负
            // 公式(9)的问题在于其不平滑，不适合大多数优化算法，因此用公式(10)替代：y(sk,sk+1) = k*<qk dk,k+1>/(1+|k*<qk dk,k+1>|)
            vel *= LogisticSigmoid(100 * (deltaS.x() * cos(conf1->GetPose().GetTheta())
                                          + deltaS.y() * sin(conf1->GetPose().GetTheta())));

            // 公式(8)
            const double omega = angle_diff / deltaT->estimate();

            // [v,w]在[vmin,vmax],[wmin,wmax]以内，error=0；否则有惩罚
            _error[0] = PenaltyBoundToInterval(vel, -config_param_->kinematics_opt().max_vel_x_backwards(),
                                               config_param_->kinematics_opt().max_vel_x(),
                                               config_param_->optimize_info().penalty_epsilon());
            _error[1] = PenaltyBoundToInterval(omega, config_param_->kinematics_opt().max_vel_theta(),
                                               config_param_->optimize_info().penalty_epsilon());
        }
    };

    class VelocityHolonomicEdge : public TebMultiEdgeBase<3, double> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VelocityHolonomicEdge() {
            this->resize(3);
        }

        void computeError() {

            const TebVertexPose *conf1 = static_cast<const TebVertexPose *>(_vertices[0]);
            const TebVertexPose *conf2 = static_cast<const TebVertexPose *>(_vertices[1]);
            const TebVertexTimeDiff *deltaT = static_cast<const TebVertexTimeDiff *>(_vertices[2]);
            const Eigen::Vector2d deltaS = conf2->estimate().GetPosition() - conf1->estimate().GetPosition();

            // 我们知道 Twk=[cos(Bk) -sin(Bk); sin(Bk) cos(Bk)] ，而Tkw=[cos(Bk) sin(Bk); -sin(Bk) cos(Bk)]
            // w：世界坐标系；    k：k时刻机器人坐标系
            // 计算车辆在k坐标系下的坐标 [r_dx,r_dy] = Tkw * dk,k+1
            double cos_theta1 = std::cos(conf1->GetPose().GetTheta());
            double sin_theta1 = std::sin(conf1->GetPose().GetTheta());
            double r_dx = cos_theta1 * deltaS.x() + sin_theta1 * deltaS.y();
            double r_dy = -sin_theta1 * deltaS.x() + cos_theta1 * deltaS.y();

            //计算vx,vy,w
            double vx = r_dx / deltaT->estimate();
            double vy = r_dy / deltaT->estimate();
            double omega = g2o::normalize_theta(conf2->GetPose().GetTheta() - conf1->GetPose().GetTheta()) /
                           deltaT->estimate();

            // [vx,vy,w]在[vxmin,vxmax],[-vymax,vymax],[wmin,wmax]以内，error=0；否则有惩罚
            _error[0] = PenaltyBoundToInterval(vx, -config_param_->kinematics_opt().max_vel_x_backwards(),
                                               config_param_->kinematics_opt().max_vel_x(),
                                               config_param_->optimize_info().penalty_epsilon());
            _error[1] = PenaltyBoundToInterval(vy, config_param_->kinematics_opt().max_vel_y(), 0.0);
            _error[2] = PenaltyBoundToInterval(omega, config_param_->kinematics_opt().max_vel_theta(),
                                               config_param_->optimize_info().penalty_epsilon());
        }
    };
}

#endif //OR_PLANNING_TEB_VELOCITY_EDGE_H
