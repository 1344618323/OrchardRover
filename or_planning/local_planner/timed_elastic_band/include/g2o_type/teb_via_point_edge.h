//
// Created by cxn on 2020/7/8.
//

#ifndef OR_PLANNING_TEB_VIA_POINT_EDGE_H
#define OR_PLANNING_TEB_VIA_POINT_EDGE_H

#include "g2o_type/teb_vertex_pose.h"
#include "teb_base_edge.h"

namespace or_local_planner
{
    /*
     * 连接一个位姿顶点的边，设定一个目标(x,y),error=||顶点(x,y) - 目标(x,y)||
     * 让链接的位姿顶点倾向于目标(x,y)
     */
    class ViaPointEdge : public TebUnaryEdgeBase<1, const Eigen::Vector2d *, TebVertexPose>
    {
    public:
        ViaPointEdge()
        {
            _measurement = NULL;
        }

        void computeError()
        {
            const TebVertexPose *bandpt = static_cast<const TebVertexPose *>(_vertices[0]);

            _error[0] = (bandpt->GetPose().GetPosition() - *_measurement).norm();
        }

        void SetViaPoint(const Eigen::Vector2d *via_point)
        {
            _measurement = via_point;
        }

        void SetParameters(const Eigen::Vector2d *via_point)
        {
            _measurement = via_point;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace or_local_planner

#endif //OR_PLANNING_TEB_VIA_POINT_EDGE_H
