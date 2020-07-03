//
// Created by cxn on 2020/7/1.
//

#include "layered_costmap.h"

namespace or_costmap {
    LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown) : costmap_(), \
                             global_frame_id_(global_frame), is_rolling_window_(rolling_window), is_initialized_(false), \
                             is_size_locked_(false), inflation_file_path_("") {
        if (track_unknown) {
            costmap_.SetDefaultValue(NO_INFORMATION);
        } else {
            costmap_.SetDefaultValue(FREE_SPACE);
        }
    }

    LayeredCostmap::~LayeredCostmap() {
        for (auto i = plugins_.size(); i > 0; i--) {
            delete plugins_[i - 1];
        }
        plugins_.clear();
    }

    //costmap是不是实时更新的，主要是障碍物层
    bool LayeredCostmap::IsCurrent() {
        is_current_ = true;
        for (auto it = plugins_.begin(); it != plugins_.end(); ++it) {
            is_current_ = is_current_ && (*it)->IsCurrent();
        }
        return is_current_;
    }

    //更新每层grid的尺寸
    void LayeredCostmap::ResizeMap(unsigned int size_x,
                                   unsigned int size_y,
                                   double resolution,
                                   double origin_x,
                                   double origin_y,
                                   bool size_locked) {
        is_size_locked_ = size_locked;
        costmap_.ResizeMap(size_x, size_y, resolution, origin_x, origin_y);
        for (auto it = plugins_.begin(); it != plugins_.end(); ++it) {
            (*it)->MatchSize();
        }
    }


    //更新master_costmap
    // 调用各层costmap的每层costmap都调用UpdateBounds，算出一个用于更新master_costmap的BoundingBox
    // 先将master_costmap BoundingBox内的cell清空，再调用每层costmap的UpdateCosts，修改BoundingBox内的cell值
    void LayeredCostmap::UpdateMap(double robot_x, double robot_y, double robot_yaw) {
        static int count = 0;
        std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.GetMutex()));
        if (is_rolling_window_) {
            double new_origin_x = robot_x - costmap_.GetSizeXWorld() / 2;
            double new_origin_y = robot_y - costmap_.GetSizeYWorld() / 2;
            costmap_.UpdateOrigin(new_origin_x, new_origin_y);
        }
        if (plugins_.size() == 0) {
            ROS_WARN("No Layer");
            return;
        }

        //每层costmap都调用UpdateBounds，算出一个用于更新的BoundingBox
        minx_ = miny_ = 1e30;
        maxx_ = maxy_ = -1e30;
        for (auto plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin) {
            double prev_minx = minx_;
            double prev_miny = miny_;
            double prev_maxx = maxx_;
            double prev_maxy = maxy_;
            (*plugin)->UpdateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
            count++;
            if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy) {
                ROS_WARN("Illegal bounds change. The offending layer is %s", (*plugin)->GetName().c_str());
            }
        }
        int x0, xn, y0, yn;
        costmap_.World2MapWithBoundary(minx_, miny_, x0, y0);
        costmap_.World2MapWithBoundary(maxx_, maxy_, xn, yn);
        x0 = std::max(0, x0);
        xn = std::min(int(costmap_.GetSizeXCell()), xn + 1);
        y0 = std::max(0, y0);
        yn = std::min(int(costmap_.GetSizeYCell()), yn + 1);
        if (xn < x0 || yn < y0) {
            return;
        }
        costmap_.ResetPartMap(x0, y0, xn, yn);
        for (auto plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin) {
            (*plugin)->UpdateCosts(costmap_, x0, y0, xn, yn);
        }

        bx0_ = x0;
        bxn_ = xn;
        by0_ = y0;
        byn_ = yn;
        is_initialized_ = true;
    }

    void LayeredCostmap::SetFootprint(const std::vector<geometry_msgs::Point> &footprint_spec) {
        footprint_ = footprint_spec;
        CalculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);
        for (auto plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin) {
            (*plugin)->OnFootprintChanged();
        }
    }
}