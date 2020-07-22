//
// Created by cxn on 2020/7/8.
//

#ifndef OR_PLANNING_TEB_PREFER_ROTDIR_EDGE_H
#define OR_PLANNING_TEB_PREFER_ROTDIR_EDGE_H

#include "g2o_type/teb_vertex_pose.h"
#include "teb_base_edge.h"
#include "teb_penalties.h"

namespace or_local_planner
{
    /*
     * 倾向于左转or右转
     * 一条边连接两个位姿顶点，若期望左转，实际位姿顶点是右转，则有惩罚，实际位姿顶点是左转，无惩罚
     *                    若期望右转，也是同理
     */

    class PreferRotDirEdge : public TebBinaryEdgeBase<1, double, TebVertexPose, TebVertexPose>
    {
    public:
        PreferRotDirEdge()
        {
            _measurement = 1;
        }

        void computeError()
        {
            const TebVertexPose *conf1 = static_cast<const TebVertexPose *>(_vertices[0]);
            const TebVertexPose *conf2 = static_cast<const TebVertexPose *>(_vertices[1]);

            _error[0] = PenaltyBoundFromBelow(_measurement * g2o::normalize_theta(conf2->GetPose().GetTheta() - conf1->GetPose().GetTheta()), 0, 0);
        }

        void SetRotDir(double dir)
        {
            _measurement = dir;
        }

        void PreferRight() { _measurement = -1; }

        void PreferLeft() { _measurement = 1; }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace or_local_planner

#endif //OR_PLANNING_TEB_PREFER_ROTDIR_EDGE_H
