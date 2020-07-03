//
// Created by cxn on 2020/7/2.
//

#ifndef OR_COSTMAP_INFLATION_LAYER_H
#define OR_COSTMAP_INFLATION_LAYER_H

#include <mutex>
#include "layer.h"
#include "layered_costmap.h"

namespace or_costmap {
    /*
     * 膨胀层
     * updateBounds()：刚确定膨胀参数的时刻，会返回一个巨大的boundingBox；
     *                  否则只是在输入的boundingBox加上膨胀半径，输出
     * updateCosts(): 在master_costmap的grid给因膨胀才有值的cell赋值
     */

    /**
     * @class CellData
     * @brief Storage for cell information used during obstacle inflation
     */
    class CellData {
    public:
        /**
         * @brief  Constructor for a CellData objects
         * @param  i The index of the cell in the cost map
         * @param  x The x coordinate of the cell in the cost map
         * @param  y The y coordinate of the cell in the cost map
         * @param  sx The x coordinate of the closest obstacle cell in the costmap
         * @param  sy The y coordinate of the closest obstacle cell in the costmap
         * @return
         */
        CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
                index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy) {
        }

        unsigned int index_;
        unsigned int x_, y_;
        unsigned int src_x_, src_y_;
    };

    class InflationLayer : public Layer {
    public:
    public:
        InflationLayer();

        virtual ~InflationLayer() {
            DeleteKernels();
        }

        virtual void OnInitialize();

        virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                  double *max_x, double *max_y);

        virtual void UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

        virtual bool IsDiscretized() {
            return true;
        }

        virtual void MatchSize();

        virtual void Reset() { OnInitialize(); }

        /** @brief  Given a distance, compute a cost.
         * @param  distance The distance from an obstacle in cells
         * @return A cost value for the distance
         * 内接圆内cell为253
         * 内接圆到膨胀半径内 cost= 252 * exp[-1.0 * weight_ * (到254cell(最近障碍物)的距离 - 内接圆半径)]
         * weight_由pb文件中cost_scaling_factor决定，
         * cost_scaling_factor越大，则衰减越快
         */
        inline unsigned char ComputeCost(unsigned distance_cell) const {
            unsigned char cost = 0;
            if (distance_cell == 0)
                cost = LETHAL_OBSTACLE;
            else if (distance_cell * resolution_ <= inscribed_radius_)
                cost = INSCRIBED_INFLATED_OBSTACLE;
            else {
                // make sure cost falls off by Euclidean distance
                double euclidean_distance = distance_cell * resolution_;
                double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
                cost = (unsigned char) ((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
            }
            return cost;
        }

        /**
         * @brief Change the values of the inflation radius parameters
         * @param inflation_radius The new inflation radius
         * @param cost_scaling_factor The new weight
         */
        void SetInflationParameters(double inflation_radius, double cost_scaling_factor);

    protected:
        virtual void OnFootprintChanged();

        std::recursive_mutex *inflation_access_;

    private:
        /**
         * @brief  Lookup pre-computed distances
         * @param mx The x coordinate of the current cell
         * @param my The y coordinate of the current cell
         * @param src_x The x coordinate of the source cell
         * @param src_y The y coordinate of the source cell
         * @return
         */
        inline double DistanceLookup(int mx, int my, int src_x, int src_y) {
            unsigned int dx = abs(mx - src_x);
            unsigned int dy = abs(my - src_y);
            return cached_distances_[dx][dy];
        }

        /**
         * @brief  Lookup pre-computed costs
         * @param mx The x coordinate of the current cell
         * @param my The y coordinate of the current cell
         * @param src_x The x coordinate of the source cell
         * @param src_y The y coordinate of the source cell
         * @return
         */
        inline unsigned char CostLookup(int mx, int my, int src_x, int src_y) {
            unsigned int dx = abs(mx - src_x);
            unsigned int dy = abs(my - src_y);
            return cached_costs_[dx][dy];
        }

        void ComputeCaches();

        void DeleteKernels();

        void InflateArea(int min_i, int min_j, int max_i, int max_j, unsigned char *master_grid);

        unsigned int CellDistance(double world_dist) {
            return layered_costmap_->GetCostMap()->World2Cell(world_dist);
        }

        inline void Enqueue(unsigned int index, unsigned int mx, unsigned int my,
                            unsigned int src_x, unsigned int src_y);

        double inflation_radius_, inscribed_radius_, weight_;//膨胀半径，单位米；内接圆，单位米；膨胀衰减权重
        bool inflate_unknown_;
        unsigned int cell_inflation_radius_;//膨胀半径，单位格
        unsigned int cached_cell_inflation_radius_;
        std::map<double, std::vector<CellData> > inflation_cells_;

        double resolution_;

        bool *seen_;
        int seen_size_;

        unsigned char **cached_costs_;
        double **cached_distances_;
        double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

        bool need_reinflation_;
    };
}

#endif //OR_COSTMAP_INFLATION_LAYER_H
