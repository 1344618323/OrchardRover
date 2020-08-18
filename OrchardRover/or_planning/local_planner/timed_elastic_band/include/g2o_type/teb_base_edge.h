//
// Created by cxn on 2020/7/6.
//

#ifndef TIMED_ELASTIC_BAND_TEB_BASE_EDGE_H
#define TIMED_ELASTIC_BAND_TEB_BASE_EDGE_H

#include <cmath>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>

/*
 * 写了三个类：单元边，二元边，多元边
 * 边的误差计算函数、存盘、读盘函数需要重写；在这个文件中，重写了存盘、读盘函数；其子类有重写误差计算函数
 */

namespace or_local_planner
{

    // 模板参数：error维度(默认类型是double)，measurement类型，连接顶点类型
    template <int D, typename E, typename VertexXi>
    class TebUnaryEdgeBase : public g2o::BaseUnaryEdge<D, E, VertexXi>
    {
    public:
        using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
        using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;

        TebUnaryEdgeBase()
        {
            _vertices[0] = NULL;
        }

        virtual ~TebUnaryEdgeBase()
        {
            if (_vertices[0])
            {
                _vertices[0]->edges().erase(this);
            }
        }

        ErrorVector &GetError()
        {
            computeError();
            return _error;
        }
        void SetConfig(const Config &config_param)
        {
            config_param_ = &config_param;
        }
        virtual bool read(std::istream &is)
        {
            return true;
        }
        virtual bool write(std::ostream &os) const
        {
            return os.good();
        }

    protected:
        using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
        using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;
        const Config *config_param_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    }; // class TebBaseEage

    // 模板参数：error维度(默认类型是double)，measurement类型，连接顶点i类型，连接顶点j类型
    template <int D, typename E, typename VertexXi, typename VertexXj>
    class TebBinaryEdgeBase : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>
    {
    public:
        using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
        using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;

        TebBinaryEdgeBase()
        {
            _vertices[0] = _vertices[1] = NULL;
        }

        virtual ~TebBinaryEdgeBase()
        {
            if (_vertices[0])
                _vertices[0]->edges().erase(this);
            if (_vertices[1])
                _vertices[1]->edges().erase(this);
        }

        ErrorVector &GetError()
        {
            computeError();
            return _error;
        }

        void SetConfig(const Config &config_param)
        {
            config_param_ = &config_param;
        }

        virtual bool read(std::istream &is)
        {
            return true;
        }

        virtual bool write(std::ostream &os) const
        {
            return os.good();
        }

    protected:
        using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
        using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;
        const Config *config_param_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    // 模板参数：error维度(默认类型是double)，measurement类型

    template <int D, typename E>
    class TebMultiEdgeBase : public g2o::BaseMultiEdge<D, E>
    {

    public:
        using typename g2o::BaseMultiEdge<D, E>::ErrorVector;
        using g2o::BaseMultiEdge<D, E>::computeError;

        TebMultiEdgeBase()
        {
        }

        virtual ~TebMultiEdgeBase()
        {
            for (std::size_t i = 0; i < _vertices.size(); ++i)
            {
                if (_vertices[i])
                    _vertices[i]->edges().erase(this);
            }
        }

        virtual void resize(size_t size)
        {
            g2o::BaseMultiEdge<D, E>::resize(size);

            for (std::size_t i = 0; i < _vertices.size(); ++i)
                _vertices[i] = NULL;
        }

        ErrorVector &GetError()
        {
            computeError();
            return _error;
        }

        void SetConfig(const Config &config_param)
        {
            config_param_ = &config_param;
        }

        virtual bool read(std::istream &is)
        {
            return true;
        }

        virtual bool write(std::ostream &os) const
        {
            return os.good();
        }

    protected:
        using g2o::BaseMultiEdge<D, E>::_error;
        using g2o::BaseMultiEdge<D, E>::_vertices;
        const Config *config_param_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace or_local_planner

#endif //TIMED_ELASTIC_BAND_TEB_BASE_EDGE_H
