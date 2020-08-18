//
// Created by cxn on 2020/7/6.
//

#ifndef OR_PLANNING_TEB_ACCELERATION_EDGE_H
#define OR_PLANNING_TEB_ACCELERATION_EDGE_H

#include <geometry_msgs/Twist.h>
#include "utility_tool.h"
#include "g2o_type/teb_vertex_pose.h"
#include "g2o_type/teb_vertex_timediff.h"
#include "teb_penalties.h"
#include "teb_base_edge.h"

namespace or_local_planner
{

    /*
     * 非完整性约束加速度边，与VelocityEdge套路一样
    */
    class AccelerationEdge : public TebMultiEdgeBase<2, double>
    {
    public:
        AccelerationEdge()
        {
            this->resize(5);
        }

        void computeError()
        {
            const TebVertexPose *pose1 = static_cast<const TebVertexPose *>(_vertices[0]);
            const TebVertexPose *pose2 = static_cast<const TebVertexPose *>(_vertices[1]);
            const TebVertexPose *pose3 = static_cast<const TebVertexPose *>(_vertices[2]);
            const TebVertexTimeDiff *dt1 = static_cast<const TebVertexTimeDiff *>(_vertices[3]);
            const TebVertexTimeDiff *dt2 = static_cast<const TebVertexTimeDiff *>(_vertices[4]);

            const Eigen::Vector2d diff1 = pose2->GetPose().GetPosition() - pose1->GetPose().GetPosition();
            const Eigen::Vector2d diff2 = pose3->GetPose().GetPosition() - pose2->GetPose().GetPosition();

            double dist1 = diff1.norm();
            double dist2 = diff2.norm();
            const double angle_diff1 = g2o::normalize_theta(pose2->GetPose().GetTheta() - pose1->GetPose().GetTheta());
            const double angle_diff2 = g2o::normalize_theta(pose3->GetPose().GetTheta() - pose2->GetPose().GetTheta());

            if (config_param_->trajectory_opt().exact_arc_length())
            {
                //与非完整性约束速度边一样，考虑转弯半半径，重算dist
                if (angle_diff1 != 0)
                {
                    const double radius = dist1 / (2 * sin(angle_diff1 / 2));
                    dist1 = fabs(angle_diff1 * radius);
                }
                if (angle_diff2 != 0)
                {
                    const double radius = dist2 / (2 * sin(angle_diff2 / 2));
                    dist2 = fabs(angle_diff2 * radius);
                }
            }

            double vel1 = dist1 / dt1->GetDiffTime();
            double vel2 = dist2 / dt2->GetDiffTime();

            vel1 *= LogisticSigmoid(100 * (diff1.x() * cos(pose1->GetPose().GetTheta()) + diff1.y() * sin(pose1->GetPose().GetTheta())));
            vel2 *= LogisticSigmoid(100 * (diff2.x() * cos(pose2->GetPose().GetTheta()) + diff2.y() * sin(pose2->GetPose().GetTheta())));

            /*
             *   公式(11)
             *   假设k时，起始速度为v0，有：dk+v0*tk+1/2*ak*tk*tk=(dk+1)
             *   假设k+1时，起始速度为v1，(dk+1)+(v1)*(tk+1)+1/2*ak*(tk+1)*(tk+1)=(dk+2)，其中v1=v0+ak*t0
             *   另外有k到k+1时，平均速度：vk=((dk+1)-dk)/tk=v0+1/2*ak*tk
             *   k+1到k+2时，平均速度：(vk+1)=((dk+2)-(dk+1))/(tk+1)=v1+1/2*ak*(tk+1)
             *   (vk+1)-vk=v1-v0+1/2*ak*((tk+1)-tk)=1/2*ak*((tk+1)+tk)
             *   所以： ak = 2[(vk+1)-vk]/((tk+1)+tk)
             */
            const double acc_lin = (vel2 - vel1) * 2 / (dt1->GetDiffTime() + dt2->GetDiffTime());

            _error[0] = PenaltyBoundToInterval(acc_lin, config_param_->kinematics_opt().acc_lim_x(),
                                               config_param_->optimize_info().penalty_epsilon());

            const double omega1 = angle_diff1 / dt1->GetDiffTime();
            const double omega2 = angle_diff2 / dt2->GetDiffTime();
            const double acc_rot = (omega2 - omega1) * 2 / (dt1->GetDiffTime() + dt2->GetDiffTime());

            _error[1] = PenaltyBoundToInterval(acc_rot, config_param_->kinematics_opt().acc_lim_theta(),
                                               config_param_->optimize_info().penalty_epsilon());
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class AccelerationStartEdge : public TebMultiEdgeBase<2, const geometry_msgs::Twist *>
    {
    public:
        AccelerationStartEdge()
        {
            _measurement = NULL;
            this->resize(3);
        }

        void computeError()
        {
            const TebVertexPose *pose1 = static_cast<const TebVertexPose *>(_vertices[0]);
            const TebVertexPose *pose2 = static_cast<const TebVertexPose *>(_vertices[1]);
            const TebVertexTimeDiff *dt = static_cast<const TebVertexTimeDiff *>(_vertices[2]);

            const Eigen::Vector2d diff = pose2->GetPose().GetPosition() - pose1->GetPose().GetPosition();
            double dist = diff.norm();
            const double angle_diff = g2o::normalize_theta(pose2->GetPose().GetTheta() - pose1->GetPose().GetTheta());
            if (config_param_->trajectory_opt().exact_arc_length() && angle_diff != 0)
            {
                const double radius = dist / (2 * sin(angle_diff / 2));
                dist = fabs(angle_diff * radius);
            }

            const double vel1 = _measurement->linear.x;
            double vel2 = dist / dt->GetDiffTime();

            vel2 *= LogisticSigmoid(100 * (diff.x() * cos(pose1->GetPose().GetTheta()) + diff.y() * sin(pose1->GetPose().GetTheta())));

            // 假设k到k+1时刻，平均速度为vk=start速度=_measurement->linear.x
            // 假设k到k+1时刻的 tk = k+1到k+2的时刻 tk+1
            // 我们只要搞出(vk+1)与(tk+1)即可
            // 所以： ak = 2[(vk+1)-vk]/((tk+1)+tk)=((vk+1)-start速度)/(tk+1)
            const double acc_lin = (vel2 - vel1) / dt->GetDiffTime();

            _error[0] = PenaltyBoundToInterval(acc_lin, config_param_->kinematics_opt().acc_lim_x(),
                                               config_param_->optimize_info().penalty_epsilon());

            const double omega1 = _measurement->angular.z;
            const double omega2 = angle_diff / dt->GetDiffTime();
            const double acc_rot = (omega2 - omega1) / dt->GetDiffTime();

            _error[1] = PenaltyBoundToInterval(acc_rot, config_param_->kinematics_opt().acc_lim_theta(),
                                               config_param_->optimize_info().penalty_epsilon());
        }

        void SetInitialVelocity(const geometry_msgs::Twist &vel_start)
        {
            _measurement = &vel_start;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    //与AccelerationStartEdge同理
    class AccelerationGoalEdge : public TebMultiEdgeBase<2, const geometry_msgs::Twist *>
    {
    public:
        AccelerationGoalEdge()
        {
            _measurement = NULL;
            this->resize(3);
        }

        void computeError()
        {
            const TebVertexPose *pose_pre_goal = static_cast<const TebVertexPose *>(_vertices[0]);
            const TebVertexPose *pose_goal = static_cast<const TebVertexPose *>(_vertices[1]);
            const TebVertexTimeDiff *dt = static_cast<const TebVertexTimeDiff *>(_vertices[2]);

            const Eigen::Vector2d diff = pose_goal->GetPose().GetPosition() - pose_pre_goal->GetPose().GetPosition();
            double dist = diff.norm();
            const double angle_diff = g2o::normalize_theta(pose_goal->GetPose().GetTheta() - pose_pre_goal->GetPose().GetTheta());
            if (config_param_->trajectory_opt().exact_arc_length() && angle_diff != 0)
            {
                double radius = dist / (2 * sin(angle_diff / 2));
                dist = fabs(angle_diff * radius);
            }

            double vel1 = dist / dt->GetDiffTime();
            const double vel2 = _measurement->linear.x;

            vel1 *= LogisticSigmoid(100 * (diff.x() * cos(pose_pre_goal->GetPose().GetTheta()) + diff.y() * sin(pose_pre_goal->GetPose().GetTheta())));

            const double acc_lin = (vel2 - vel1) / dt->GetDiffTime();

            _error[0] = PenaltyBoundToInterval(acc_lin, config_param_->kinematics_opt().acc_lim_x(),
                                               config_param_->optimize_info().penalty_epsilon());

            const double omega1 = angle_diff / dt->GetDiffTime();
            const double omega2 = _measurement->angular.z;
            const double acc_rot = (omega2 - omega1) / dt->GetDiffTime();

            _error[1] = PenaltyBoundToInterval(acc_rot, config_param_->kinematics_opt().acc_lim_theta(),
                                               config_param_->optimize_info().penalty_epsilon());
        }

        void SetGoalVelocity(const geometry_msgs::Twist &vel_goal)
        {
            _measurement = &vel_goal;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /*
     * 完整性约束加速度边，与VelocityHolonomicEdge套路一样
     */

    class AccelerationHolonomicEdge : public TebMultiEdgeBase<3, double>
    {
    public:
        AccelerationHolonomicEdge()
        {
            this->resize(5);
        }

        void computeError()
        {
            const TebVertexPose *pose1 = static_cast<const TebVertexPose *>(_vertices[0]);
            const TebVertexPose *pose2 = static_cast<const TebVertexPose *>(_vertices[1]);
            const TebVertexPose *pose3 = static_cast<const TebVertexPose *>(_vertices[2]);
            const TebVertexTimeDiff *dt1 = static_cast<const TebVertexTimeDiff *>(_vertices[3]);
            const TebVertexTimeDiff *dt2 = static_cast<const TebVertexTimeDiff *>(_vertices[4]);

            Eigen::Vector2d diff1 = pose2->GetPose().GetPosition() - pose1->GetPose().GetPosition();
            Eigen::Vector2d diff2 = pose3->GetPose().GetPosition() - pose2->GetPose().GetPosition();

            double cos_theta1 = std::cos(pose1->GetPose().GetTheta());
            double sin_theta1 = std::sin(pose1->GetPose().GetTheta());
            double cos_theta2 = std::cos(pose2->GetPose().GetTheta());
            double sin_theta2 = std::sin(pose2->GetPose().GetTheta());

            double p1_dx = cos_theta1 * diff1.x() + sin_theta1 * diff1.y();
            double p1_dy = -sin_theta1 * diff1.x() + cos_theta1 * diff1.y();

            double p2_dx = cos_theta2 * diff2.x() + sin_theta2 * diff2.y();
            double p2_dy = -sin_theta2 * diff2.x() + cos_theta2 * diff2.y();

            double vel1_x = p1_dx / dt1->GetDiffTime();
            double vel1_y = p1_dy / dt1->GetDiffTime();
            double vel2_x = p2_dx / dt2->GetDiffTime();
            double vel2_y = p2_dy / dt2->GetDiffTime();

            double dt12 = dt1->GetDiffTime() + dt2->GetDiffTime();

            double acc_x = (vel2_x - vel1_x) * 2 / dt12;
            double acc_y = (vel2_y - vel1_y) * 2 / dt12;

            _error[0] = PenaltyBoundToInterval(acc_x, config_param_->kinematics_opt().acc_lim_x(),
                                               config_param_->optimize_info().penalty_epsilon());
            _error[1] = PenaltyBoundToInterval(acc_y, config_param_->kinematics_opt().acc_lim_y(),
                                               config_param_->optimize_info().penalty_epsilon());

            double omega1 = g2o::normalize_theta(pose2->GetPose().GetTheta() - pose1->GetPose().GetTheta()) / dt1->GetDiffTime();
            double omega2 = g2o::normalize_theta(pose3->GetPose().GetTheta() - pose2->GetPose().GetTheta()) / dt2->GetDiffTime();
            double acc_rot = (omega2 - omega1) * 2 / dt12;

            _error[2] = PenaltyBoundToInterval(acc_rot, config_param_->kinematics_opt().acc_lim_theta(),
                                               config_param_->optimize_info().penalty_epsilon());
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class AccelerationHolonomicStartEdge : public TebMultiEdgeBase<3, const geometry_msgs::Twist *>
    {
    public:
        AccelerationHolonomicStartEdge()
        {
            this->resize(3);
            _measurement = NULL;
        }

        void computeError()
        {
            const TebVertexPose *pose1 = static_cast<const TebVertexPose *>(_vertices[0]);
            const TebVertexPose *pose2 = static_cast<const TebVertexPose *>(_vertices[1]);
            const TebVertexTimeDiff *dt = static_cast<const TebVertexTimeDiff *>(_vertices[2]);

            Eigen::Vector2d diff = pose2->GetPose().GetPosition() - pose1->GetPose().GetPosition();

            double cos_theta1 = std::cos(pose1->GetPose().GetTheta());
            double sin_theta1 = std::sin(pose1->GetPose().GetTheta());

            double p1_dx = cos_theta1 * diff.x() + sin_theta1 * diff.y();
            double p1_dy = -sin_theta1 * diff.x() + cos_theta1 * diff.y();

            double vel1_x = _measurement->linear.x;
            double vel1_y = _measurement->linear.y;
            double vel2_x = p1_dx / dt->GetDiffTime();
            double vel2_y = p1_dy / dt->GetDiffTime();

            double acc_lin_x = (vel2_x - vel1_x) / dt->GetDiffTime();
            double acc_lin_y = (vel2_y - vel1_y) / dt->GetDiffTime();

            _error[0] = PenaltyBoundToInterval(acc_lin_x, config_param_->kinematics_opt().acc_lim_x(),
                                               config_param_->optimize_info().penalty_epsilon());
            _error[1] = PenaltyBoundToInterval(acc_lin_y, config_param_->kinematics_opt().acc_lim_y(),
                                               config_param_->optimize_info().penalty_epsilon());

            double omega1 = _measurement->angular.z;
            double omega2 = g2o::normalize_theta(pose2->GetPose().GetTheta() - pose1->GetPose().GetTheta()) / dt->GetDiffTime();
            double acc_rot = (omega2 - omega1) / dt->GetDiffTime();

            _error[2] = PenaltyBoundToInterval(acc_rot, config_param_->kinematics_opt().acc_lim_theta(),
                                               config_param_->optimize_info().penalty_epsilon());
        }

        void setInitialVelocity(const geometry_msgs::Twist &vel_start)
        {
            _measurement = &vel_start;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class AccelerationHolonomicGoalEdge : public TebMultiEdgeBase<3, const geometry_msgs::Twist *>
    {
    public:
        AccelerationHolonomicGoalEdge()
        {
            _measurement = NULL;
            this->resize(3);
        }

        void computeError()
        {

            const TebVertexPose *pose_pre_goal = static_cast<const TebVertexPose *>(_vertices[0]);
            const TebVertexPose *pose_goal = static_cast<const TebVertexPose *>(_vertices[1]);
            const TebVertexTimeDiff *dt = static_cast<const TebVertexTimeDiff *>(_vertices[2]);

            Eigen::Vector2d diff = pose_goal->GetPose().GetPosition() - pose_pre_goal->GetPose().GetPosition();

            double cos_theta1 = std::cos(pose_pre_goal->GetPose().GetTheta());
            double sin_theta1 = std::sin(pose_pre_goal->GetPose().GetTheta());

            double p1_dx = cos_theta1 * diff.x() + sin_theta1 * diff.y();
            double p1_dy = -sin_theta1 * diff.x() + cos_theta1 * diff.y();

            double vel1_x = p1_dx / dt->GetDiffTime();
            double vel1_y = p1_dy / dt->GetDiffTime();
            double vel2_x = _measurement->linear.x;
            double vel2_y = _measurement->linear.y;

            double acc_lin_x = (vel2_x - vel1_x) / dt->GetDiffTime();
            double acc_lin_y = (vel2_y - vel1_y) / dt->GetDiffTime();

            _error[0] = PenaltyBoundToInterval(acc_lin_x, config_param_->kinematics_opt().acc_lim_x(),
                                               config_param_->optimize_info().penalty_epsilon());
            _error[1] = PenaltyBoundToInterval(acc_lin_x, config_param_->kinematics_opt().acc_lim_y(),
                                               config_param_->optimize_info().penalty_epsilon());

            double omega1 = g2o::normalize_theta(pose_goal->GetPose().GetTheta() - pose_pre_goal->GetPose().GetTheta()) / dt->GetDiffTime();
            double omega2 = _measurement->angular.z;
            double acc_rot = (omega2 - omega1) / dt->GetDiffTime();

            _error[2] = PenaltyBoundToInterval(acc_rot, config_param_->kinematics_opt().acc_lim_theta(),
                                               config_param_->optimize_info().penalty_epsilon());
        }

        void SetGoalVelocity(const geometry_msgs::Twist &vel_goal)
        {
            _measurement = &vel_goal;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace or_local_planner

#endif //OR_PLANNING_TEB_ACCELERATION_EDGE_H
