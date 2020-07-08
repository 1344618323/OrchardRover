//
// Created by cxn on 2020/7/6.
//

#ifndef OR_PLANNING_TEB_VERTEX_TIMEDIFF_H
#define OR_PLANNING_TEB_VERTEX_TIMEDIFF_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include <Eigen/Core>

namespace or_local_planner {
    /*
     * 时间顶点
     * g2o::BaseVertex子类的更新函数、重置函数、存盘、读盘函数需要重写
     */
    class TebVertexTimeDiff : public g2o::BaseVertex<1, double> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TebVertexTimeDiff(bool fixed = false) {
            setToOriginImpl();
            setFixed(fixed);
        }

        TebVertexTimeDiff(double dt, bool fixed = false) {
            _estimate = dt;
            setFixed(fixed);
        }

        ~TebVertexTimeDiff() {}

        double &GetDiffTime() { return _estimate; }

        const double &GetDiffTime() const { return _estimate; }

        virtual void setToOriginImpl() {
            _estimate = 0.1;
        }

        virtual void oplusImpl(const double *update) {
            _estimate += *update;
        }

        virtual bool read(std::istream &is) {

        }

        virtual bool write(std::ostream &os) const {
            return true;
        }
    };

} // namespace or_local_planner

#endif //OR_PLANNING_TEB_VERTEX_TIMEDIFF_H
