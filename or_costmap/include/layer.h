//
// Created by cxn on 2020/7/1.
//

#ifndef ROBORTS_COSTMAP_LAYER_H
#define ROBORTS_COSTMAP_LAYER_H

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "layered_costmap.h"

namespace or_costmap {
    class LayeredCostmap;

    class Layer {
    public:
        /**
          * @brief constructor
          */
        Layer();

        /**
         * @brief initialize
         * @param parent the layered costmap, ie master grid
         * @param name this layer name
         * @param tf a tf listener providing transforms
         * 获取 master_costmap 的指针、给这层costmap取名，获取tf
         */
        void Initialize(LayeredCostmap *parent, std::string name, tf::TransformListener *tf);

        /**
         * @brief This is called by the LayeredCostmap to poll this plugin as to how
         *        much of the costmap it needs to update. Each layer can increase
         *        the size of this bounds. *
         * @param robot_x
         * @param robot_y
         * @param robot_yaw these point the pose of the robot in global frame
         * @param min_x
         * @param min_y
         * @param max_x
         * @param max_y these declare the updating boundary
         */
        virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                  double *max_x, double *max_y) {}

        /**
         * @brief Actually update the underlying costmap, only within the bounds
         *        calculated during UpdateBounds(). *
         * @param master_grid the master map
         * @param min_i
         * @param min_j
         * @param max_i
         * @param max_j the update boundary
         */
        virtual void UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {}

        /**
         * @brief Stop.
         */
        virtual void Deactivate() {}

        /**
         * @brief Restart, if they've been stopped.
         */
        virtual void Activate() {}

        /**
         * @brief Reset the layer
         */
        virtual void Reset() {}

        virtual ~Layer() {}

        /**
         * @brief Check to make sure all the data in the layer is update.
         * @return Whether the data in the layer is up to date.
         */
        bool IsCurrent() const {
            return is_current_;
        }

        /**
         * @brief Implement this to make this layer match the size of the parent costmap.
         * 根据 parent costmap的尺寸，重新分配这层layer对应的尺寸，使尺寸相等
         * Layer中并没有进行实现
         * 而在costmap_layer中实现了
         */
        virtual void MatchSize() {}

        std::string GetName() const {
            return name_;
        }

        /**
         * @brief Convenience function for layered_costmap_->GetFootprint().
         * 从maser_costmap处获取footprint
         */
        const std::vector<geometry_msgs::Point> &GetFootprint() const;

        virtual void OnFootprintChanged() {}

    protected:
        /** @brief This is called at the end of initialize().  Override to
         * implement subclass-specific initialization.
         * tf_, name_, and layered_costmap_ will all be set already when this is called. */
        virtual void OnInitialize() {}

        LayeredCostmap *layered_costmap_;
        bool is_current_, is_enabled_;
        std::string name_;
        tf::TransformListener *tf_;

    private:
        std::vector<geometry_msgs::Point> footprint_spec_;
    };
}

#endif //ROBORTS_COSTMAP_LAYER_H