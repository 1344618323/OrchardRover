//
// Created by cxn on 2020/7/1.
//

#ifndef OR_COSTMAP_STATIC_LYAER_H
#define OR_COSTMAP_STATIC_LYAER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "costmap_layer.h"
#include "static_layer_setting.pb.h"
#include "proto_io/io.h"
#include <ros/package.h>

namespace or_costmap {
    /*
     * 这个类是静态层类，与输入的地图保持一致
     * 会接受地图的topic，在接受到topic后，
     *                       若master_costmap是不动的，会让master_costmap调用各层的MatchSize(),调整各层的grid尺寸
     *                       若master_costmap是移动的，之会调整这层的grid尺寸
     *                       此外，还会将地图topic中内容复制到这层grid上(即这层grid的尺寸、原点都与地图topic的相同，无论master_costmap是不是rolling)
     * updateBounds():除了地图的topic的尺寸发生变换时，会输出整图尺寸作为boundingBox
     *                                           否则，会将输入的boundingBox原封不动地输出
     * updateCosts():若master_costmap是不动的，将这层grid boundingBox 中的内容复制到 master_costmap的boundingBox中
     *               若master_costmap是移动的，求boundingBox在这层grid中的坐标，在将里面的内容复制到master_costmap中
     */
    class StaticLayer : public CostmapLayer {
    public:
        StaticLayer() {}

        virtual ~StaticLayer() {}

        virtual void OnInitialize();

        virtual void Activate();

        virtual void Deactivate();

        virtual void Reset();

        virtual void UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

        virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                  double *max_x, double *max_y);

        virtual void MatchSize();

    private:
        void InComingMap(const nav_msgs::OccupancyGridConstPtr &new_map);

        unsigned char InterpretValue(unsigned char value);

        std::string global_frame_;
        std::string map_frame_;
        std::string map_topic_;
        bool subscribe_to_updates_;
        bool map_received_;
        bool has_updated_data_;
        unsigned int staic_layer_x_, staic_layer_y_, width_, height_;
        unsigned char lethal_threshold_, unknown_cost_value_;
        bool track_unknown_space_;
        bool use_maximum_;
        bool first_map_only_;
        bool trinary_costmap_;
        ros::Subscriber map_sub_, map_update_sub_;
    };
}

#endif //OR_COSTMAP_STATIC_LYAER_H
