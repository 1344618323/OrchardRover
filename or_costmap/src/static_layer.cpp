//
// Created by cxn on 2020/7/1.
//

#include "static_layer.h"

namespace or_costmap {
    void StaticLayer::OnInitialize() {
        ros::NodeHandle nh;
        is_current_ = true;
        ParaStaticLayer para_static_layer;
        or_io::ReadProtoFromTextFile(layered_costmap_->GetStaticFilePath().c_str(), &para_static_layer);
        global_frame_ = layered_costmap_->GetGlobalFrameID();
        first_map_only_ = para_static_layer.first_map_only();
        subscribe_to_updates_ = para_static_layer.subscribe_to_updates();
        track_unknown_space_ = para_static_layer.track_unknown_space();
        use_maximum_ = para_static_layer.use_maximum();
        int temp_threshold = para_static_layer.lethal_threshold();
        lethal_threshold_ = std::max(std::min(100, temp_threshold), 0);
        trinary_costmap_ = para_static_layer.trinary_map();
        unknown_cost_value_ = para_static_layer.unknown_cost_value();
        map_received_ = false;
        map_topic_ = para_static_layer.topic_name();
        map_sub_ = nh.subscribe(map_topic_.c_str(), 1, &StaticLayer::InComingMap, this);
        ros::Rate temp_rate(10);
        while (!map_received_) {
            ros::spinOnce();
            temp_rate.sleep();
        }
        staic_layer_x_ = staic_layer_y_ = 0;
        width_ = size_x_;
        height_ = size_y_;
        is_enabled_ = true;
        has_updated_data_ = true;
    }

    void StaticLayer::MatchSize() {
        if (!layered_costmap_->IsRolling()) {
            //只有master grid不是rolling的时候，才会调用这个函数
            Costmap2D *master = layered_costmap_->GetCostMap();
            ResizeMap(master->GetSizeXCell(), master->GetSizeYCell(), master->GetResolution(),
                      master->GetOriginX(), master->GetOriginY());
        }
    }

    //ros接受地图topic回调
    void StaticLayer::InComingMap(const nav_msgs::OccupancyGridConstPtr &new_map) {
        unsigned int temp_index = 0;
        unsigned char value = 0;
        unsigned int size_x = new_map->info.width, size_y = new_map->info.height;
        auto resolution = new_map->info.resolution;
        auto origin_x = new_map->info.origin.position.x;
        auto origin_y = new_map->info.origin.position.y;
        auto master_map = layered_costmap_->GetCostMap();
        if (!layered_costmap_->IsRolling() &&
            (master_map->GetSizeXCell() != size_x || master_map->GetSizeYCell() != size_y ||
             master_map->GetResolution() != resolution || master_map->GetOriginX() != origin_x ||
             master_map->GetOriginY() != origin_y ||
             !layered_costmap_->IsSizeLocked())) {
            //如果master grid不是rolling的，那么在接受新ros地图时，会修正master grid的尺寸
            //master grid尺寸修改后，会调用各层layer的MatchSize()，修正各层layer grid的尺寸
            layered_costmap_->ResizeMap(size_x, size_y, resolution, origin_x, origin_y, true);
        } else if (size_x_ != size_x || size_y_ != size_y || resolution_ != resolution || origin_x_ != origin_x ||
                   origin_y_ != origin_y) {
            //如果master grid是移动的，只修正这层layer的grid尺寸
            ResizeMap(size_x, size_y, resolution, origin_x, origin_y);
        }

        for (auto i = 0; i < size_y; i++) {
            for (auto j = 0; j < size_x; j++) {
                value = new_map->data[temp_index];
                costmap_[temp_index] = InterpretValue(value);
                ++temp_index;
            }
        }
        map_received_ = true;
        has_updated_data_ = true;
        map_frame_ = new_map->header.frame_id;
        staic_layer_x_ = staic_layer_y_ = 0;
        width_ = size_x_;
        height_ = size_y_;
        if (first_map_only_) {
            map_sub_.shutdown();
        }
    }

    unsigned char StaticLayer::InterpretValue(unsigned char value) {
        // check if the static value is above the unknown or lethal thresholds
        if (track_unknown_space_ && value == unknown_cost_value_)
            return NO_INFORMATION;
        else if (!track_unknown_space_ && value == unknown_cost_value_)
            return FREE_SPACE;
        else if (value >= lethal_threshold_)
            return LETHAL_OBSTACLE;
        else if (trinary_costmap_)
            return FREE_SPACE;

        double scale = (double) value / lethal_threshold_;
        return scale * LETHAL_OBSTACLE;
    }

    void StaticLayer::Activate() {
        OnInitialize();
    }

    void StaticLayer::Deactivate() {
        //    delete cost_map_;
        //shut down the map topic message subscriber
        map_sub_.shutdown();
    }

    void StaticLayer::Reset() {
        if (first_map_only_) {
            has_updated_data_ = true;
        } else {
            OnInitialize();
        }
    }


    //如果grid不动，且（没收到地图topic || grid已经更新过了（只有初始化、重置、收到地图topic时，才会更新） || has_extra_bounds_)
    //就直接退出；
    //否则返回地图grid的尺寸
    void StaticLayer::UpdateBounds(double robot_x,
                                   double robot_y,
                                   double robot_yaw,
                                   double *min_x,
                                   double *min_y,
                                   double *max_x,
                                   double *max_y) {
        double wx, wy;
        if (!layered_costmap_->IsRollingWindow()) {
            if (!map_received_ || !(has_updated_data_ || has_extra_bounds_)) {
                return;
            }
        }
        //just make sure the value is normal
        UseExtraBounds(min_x, min_y, max_x, max_y);
        Map2World(staic_layer_x_, staic_layer_y_, wx, wy);
        *min_x = std::min(wx, *min_x);
        *min_y = std::min(wy, *min_y);
        Map2World(staic_layer_x_ + width_, staic_layer_y_ + height_, wx, wy);
        *max_x = std::max(*max_x, wx);
        *max_y = std::max(*max_y, wy);
        has_updated_data_ = false;
    }

    void StaticLayer::UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
        if (!map_received_) {
            return;
        }
        if (!layered_costmap_->IsRollingWindow()) {
            if (!use_maximum_) {
                UpdateOverwriteByAll(master_grid, min_i, min_j, max_i, max_j);
            } else {
                UpdateOverwriteByMax(master_grid, min_i, min_j, max_i, max_j);
            }
        } else {
            unsigned int mx, my;
            double wx, wy;
            tf::StampedTransform temp_transform;
            try {
                tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0), temp_transform);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                return;
            }
            for (auto i = min_i; i < max_i; ++i) {
                for (auto j = min_j; j < max_j; ++j) {
                    //得到master grid的坐标 Pglobal 转成(i,j)
                    layered_costmap_->GetCostMap()->Map2World(i, j, wx, wy);
                    tf::Point p(wx, wy, 0);
                    //求Pglobal在map中的坐标，Tmap_global*Pglobal=Pmap  转成(mx,my)
                    p = temp_transform(p);
                    if (World2Map(p.x(), p.y(), mx, my)) {
                        if (!use_maximum_) {
                            master_grid.SetCost(i, j, GetCost(mx, my));
                        } else {
                            master_grid.SetCost(i, j, std::max(master_grid.GetCost(i, j), GetCost(mx, my)));
                        }
                    }
                }
            }
        }
    }
}

