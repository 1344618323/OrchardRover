#ifndef OR_PLANNING_LOCAL_PLANNER_TEB_TIME_OPTIMAL_EDGE_H
#define OR_PLANNING_LOCAL_PLANNER_TEB_TIME_OPTIMAL_EDGE_H

#include <Eigen/Core>
#include "g2o_type/teb_vertex_timediff.h"
#include "teb_base_edge.h"
#include "teb_penalties.h"

#define USE_ANALYTIC_JACOBI

namespace or_local_planner
{
  /*(cxn)
时间边
 我们希望耗时越短越好，最好是0
其雅克比矩阵为 [1]
*/
  class TimeOptimalEdge : public TebUnaryEdgeBase<1, double, TebVertexTimeDiff>
  {
  public:
    TimeOptimalEdge()
    {
      this->setMeasurement(0.);
    }

    void computeError()
    {
      const TebVertexTimeDiff *timediff = static_cast<const TebVertexTimeDiff *>(_vertices[0]);

      _error[0] = timediff->GetDiffTime();
    }

#ifdef USE_ANALYTIC_JACOBI

    void linearizeOplus()
    {
      _jacobianOplusXi(0, 0) = 1;
    }
#endif

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace or_local_planner

#endif