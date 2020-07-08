//
// Created by cxn on 2020/7/6.
//

#ifndef OR_PLANNING_TEB_VERTEX_POSE_H
#define OR_PLANNING_TEB_VERTEX_POSE_H

#include "g2o/config.h"
#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/stuff/misc.h>
#include "data_base.h"

/*
 * SE(2)顶点类
 */

namespace or_local_planner {
    //模板参数：优化变量维度和数据类型
    //g2o::BaseVertex子类的更新函数、重置函数、存盘、读盘函数需要重写
    class TebVertexPose : public g2o::BaseVertex<3, DataBase> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TebVertexPose(bool fixed = false) {
            setToOriginImpl();
            setFixed(fixed);
        }

        TebVertexPose(const DataBase &teb_data, bool fixed = false) {
            _estimate = teb_data;
            setFixed(fixed);
        }

        ~TebVertexPose() {}

        //重置
        virtual void setToOriginImpl() {
            //_estimate是or_planning::DataBase类的对象
            //调用了or_planning::DataBase类的SetZero()
            _estimate.SetZero();
        }

        //更新
        virtual void oplusImpl(const double *update) {
            //调用了or_planning::DataBase类的Plus()
            _estimate.Plus(update);
        }

        virtual bool read(std::istream &is) {
            return true;
        }

        virtual bool write(std::ostream &os) const {
            return true;
        }

        DataBase &GetPose() {
            return _estimate;
        }

        const DataBase &GetPose() const {
            return _estimate;
        }
    };
}

#endif //OR_PLANNING_TEB_VERTEX_POSE_H
