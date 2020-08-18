//
// Created by cxn on 2020/7/8.
//

#ifndef OR_PLANNING_TEB_OBSTACLE_EDGE_H
#define OR_PLANNING_TEB_OBSTACLE_EDGE_H

#include "obstacle.h"
#include "robot_footprint_model.h"
#include "teb_base_edge.h"

namespace or_local_planner
{
    /*(cxn)障碍物边
        obstacleDist为机器人到障碍物的最小距离要求
        dist表示机器人到最近障碍物的距离
        smallEpsilon是为了安全起见加的小值
        err = (dist > (obstacleDist + smallEpsilon)) ? 0 : (obstacleDist + smallEpsilon - dist)
        相当于如果 dist比obstacleDist小，就加惩罚；否则就没有惩罚
        */
    class ObstacleEdge : public TebUnaryEdgeBase<1, const Obstacle *, TebVertexPose>
    {
    public:
        ObstacleEdge()
        {
            _measurement = NULL;
        }

        void computeError()
        {
            const TebVertexPose *bandpt = static_cast<const TebVertexPose *>(_vertices[0]);

            double dist = robot_model_->CalculateDistance(bandpt->GetPose(), _measurement);

            _error[0] = PenaltyBoundFromBelow(dist, config_param_->obstacles_opt().min_obstacle_dist(),
                                              config_param_->optimize_info().penalty_epsilon());
        }

        void SetObstacle(const Obstacle *obstacle)
        {
            _measurement = obstacle;
        }
        void SetRobotModel(const BaseRobotFootprintModel *robot_model)
        {
            robot_model_ = robot_model;
        }

        void SetParameters(const Config &config_param, const BaseRobotFootprintModel *robot_model, const Obstacle *obstacle)
        {
            config_param_ = &config_param;
            robot_model_ = robot_model;
            _measurement = obstacle;
        }

    protected:
        const BaseRobotFootprintModel *robot_model_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace or_local_planner
#endif //OR_PLANNING_TEB_OBSTACLE_EDGE_H
