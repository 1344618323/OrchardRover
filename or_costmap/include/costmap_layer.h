//
// Created by cxn on 2020/7/1.
//

#ifndef OR_COSTMAP_COSTMAP_LAYER_H
#define OR_COSTMAP_COSTMAP_LAYER_H

#include <ros/ros.h>
#include "layer.h"
#include "layered_costmap.h"

namespace or_costmap {
    class CostmapLayer : public Layer, public Costmap2D {
    public:
        CostmapLayer() : has_extra_bounds_(false),
                         extra_min_x_(1e6), extra_max_x_(-1e6),
                         extra_min_y_(1e6), extra_max_y_(-1e6) {}

        virtual void MatchSize();

        /**
          * If an external source changes values in the costmap,
          * it should call this method with the area that it changed
          * to ensure that the costmap includes this region as well.
          * @param mx0 Minimum x value of the bounding box
          * @param my0 Minimum y value of the bounding box
          * @param mx1 Maximum x value of the bounding box
          * @param my1 Maximum y value of the bounding box
          * 让boundingBox E[extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_]
          * 一般情况下除了回调（地图topic、scan topic）外，是没有东西能让costmapLayer的grid发生变化的
          * 但是也有意外情况，如果有其他情况让其发生了变化，应该记录发生变化的boundingBox E，
          * 这样在这层costmapLayer调用updatedBounds时，输出的boundingBox Out才能让包含E
          *
          * 包含另外一个boundingBox: in[mx0,  my0, mx1, my1]，得到新的E
          * E会接下来会在UseExtraBounds函数中发挥作用，使输出boundingBox Out能够包含E
          * Out赋值成功后，extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_恢复原值
          * 即分别为extra_min_x_ = 1e6;extra_min_y_ = 1e6;extra_max_x_ = -1e6;extra_max_y_ = -1e6;
          */
        void AddExtraBounds(double mx0, double my0, double mx1, double my1);

    protected:
        /*
         * Updates the master_grid within the specified
         * bounding box using this layer's values.
         *
         * All means every value from this layer
         * is written into the master grid.
         * 在限定范围内，将这层grid的值作为master的值
         */
        void UpdateOverwriteByAll(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

        /*
         * Updates the master_grid within the specified
         * bounding box using this layer's values.
         *
         * Valid means every valid value from this layer
         * is written into the master grid (does not copy NO_INFORMATION)
         * 在限定范围内，将这层grid的值作为master的值，除了这层grid的未知值
         */
        void UpdateOverwriteByValid(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

        /*
         * Updates the master_grid within the specified
         * bounding box using this layer's values.
         *
         * Sets the new value to the maximum of the master_grid's value
         * and this layer's value. If the master value is NO_INFORMATION,
         * it is overwritten. If the layer's value is NO_INFORMATION,
         * the master value does not change.
         * 在限定范围内，修改master grid的值，master.val=max(master.val,this.val);
         * 若master.val未知，就master.val赋值为this->val;若this.val未知，则master.val不变
         */
        void UpdateOverwriteByMax(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

        /*
         * Updates the master_grid within the specified
         * bounding box using this layer's values.
         *
         * Sets the new value to the sum of the master grid's value
         * and this layer's value. If the master value is NO_INFORMATION,
         * it is overwritten with the layer's value. If the layer's value
         * is NO_INFORMATION, then the master value does not change.
         *
         * If the sum value is larger than INSCRIBED_INFLATED_OBSTACLE,
         * the master value is set to (INSCRIBED_INFLATED_OBSTACLE - 1).
         *
         * 在限定范围内，修改master grid的值，master.val= master.val+ this.val;
         * 若master.val未知，就master.val赋值为this->val;若this.val未知，则master.val不变
         * 若最终master.val超过253，说明 master grid 已经膨胀到了极限，赋值为252
         */
        void UpdateOverwriteByAdd(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

        /**
         * Updates the bounding box specified in the parameters to include
         * the location (x,y)
         *
         * @param x x-coordinate to include
         * @param y y-coordinate to include
         * @param min_x bounding box
         * @param min_y bounding box
         * @param max_x bounding box
         * @param max_y bounding box
         * 让boundingBox能包含(x,y)
         */
        void Touch(double x, double y, double *min_x, double *min_y, double *max_x, double *max_y);

        /*
         * Updates the bounding box specified in the parameters
         * to include the bounding box from the addExtraBounds
         * call. If addExtraBounds was not called, the method will do nothing.
         *
         * Should be called at the beginning of the updateBounds method
         *
         * @param min_x bounding box (input and output)
         * @param min_y bounding box (input and output)
         * @param max_x bounding box (input and output)
         * @param max_y bounding box (input and output)
         * 让boundingBox Out[min_x, min_y, max_x, max_y]能够包含
         * boundingBox: E[extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_]
         * Out赋值成功后，extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_恢复原值
         * 即分别为extra_min_x_ = 1e6;extra_min_y_ = 1e6;extra_max_x_ = -1e6;extra_max_y_ = -1e6;此时E是空的
         * 当然如果boundingBox E是原值，即没有通过AddExtraBounds扩张过E，Out保持输入原值不做更改输出
         */
        void UseExtraBounds(double *min_x, double *min_y, double *max_x, double *max_y);

        bool has_extra_bounds_;

    private:
        double extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_;

    };
}
#endif //OR_COSTMAP_COSTMAP_LAYER_H
