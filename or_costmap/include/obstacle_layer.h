//
// Created by cxn on 2020/7/2.
//

#ifndef OR_COSTMAP_OBSTACLE_LAYER_H
#define OR_COSTMAP_OBSTACLE_LAYER_H

#include <chrono>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include "footprint.h"
#include "costmap_layer.h"
#include "layered_costmap.h"
#include "observation_buffer.h"

namespace or_costmap {
    /* 障碍物层，其grid尺寸是不变的,若master_costmap是rolling的，就随master_costmap的尺寸(pb文件设定)
     *                          若master_costmap不是rolling的，就随地图topic的的尺寸
     * 通过 scan topic 收集点云信息，并适时将过时的点云去掉，并没有对grid做处理
     * 比较聪明的是，它只有在被master_costmap调用updateBound时，才会将点云信息用于重置其grid
     * updateBounds():若master_costmap是rolling的,动起来
     *                若master_costmap不是rolling的,每过2s，重置grid
     *                将累积的点云：射线经过的cell置0，射线hit的cell置254
     */
    class ObstacleLayer : public CostmapLayer {
    public:
        ObstacleLayer() {
            costmap_ = nullptr;
        }

        virtual ~ObstacleLayer() {}

        virtual void OnInitialize();

        virtual void Activate();

        virtual void Deactivate();

        virtual void Reset();

        virtual void UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

        virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                  double *max_x, double *max_y) override;

        void LaserScanCallback(const sensor_msgs::LaserScanConstPtr &message,
                               const std::shared_ptr<ObservationBuffer> &buffer);

        void LaserScanValidInfoCallback(const sensor_msgs::LaserScanConstPtr &message,
                                        const std::shared_ptr<ObservationBuffer> &buffer);

    protected:
        bool GetMarkingObservations(std::vector<Observation> &marking_observations) const;

        bool GetClearingObservations(std::vector<Observation> &clearing_observations) const;

        virtual void RaytraceFreespace(const Observation &clearing_observation, double *min_x, double *min_y,
                                       double *max_x, double *max_y);

        void
        UpdateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double *min_x, double *min_y,
                             double *max_x, double *max_y);

        void UpdateFootprint(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                             double *max_x, double *max_y);

        bool footprint_clearing_enabled_, rolling_window_;
        int combination_method_;
        std::string global_frame_;
        double max_obstacle_height_;
        std::vector<geometry_msgs::Point> transformed_footprint_;
        laser_geometry::LaserProjection projector_;

        std::vector<std::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;
        std::vector<std::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;
        std::vector<std::shared_ptr<ObservationBuffer> > observation_buffers_;
        std::vector<std::shared_ptr<ObservationBuffer> > marking_buffers_;
        std::vector<std::shared_ptr<ObservationBuffer> > clearing_buffers_;

        std::vector<Observation> static_clearing_observations_, static_marking_observations_;
        std::chrono::system_clock::time_point reset_time_;
    };
}
#endif //OR_COSTMAP_OBSTACLE_LAYER_H
