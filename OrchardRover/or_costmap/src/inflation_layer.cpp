//
// Created by cxn on 2020/7/2.
//

#include "inflation_layer.h"
#include "inflation_layer_setting.pb.h"
#include "proto_io/io.h"

namespace or_costmap {
    InflationLayer::InflationLayer()
            : inflation_radius_(0),
              weight_(0),
              inflate_unknown_(false),
              cell_inflation_radius_(0),
              cached_cell_inflation_radius_(0),
              seen_(NULL),
              cached_costs_(NULL),
              cached_distances_(NULL),
              last_min_x_(-std::numeric_limits<float>::max()),
              last_min_y_(-std::numeric_limits<float>::max()),
              last_max_x_(std::numeric_limits<float>::max()),
              last_max_y_(std::numeric_limits<float>::max()) {
        inflation_access_ = new std::recursive_mutex();
    }

    void InflationLayer::OnInitialize() {
        std::unique_lock<std::recursive_mutex> lock(*inflation_access_);
        is_current_ = true;
        if (seen_)
            delete[] seen_;
        seen_ = NULL;
        seen_size_ = 0;
        need_reinflation_ = false;
        double inflation_radius, cost_scaling_factor;
        ParaInflationLayer para_inflation;
        or_common::ReadProtoFromTextFile(layered_costmap_->GetInflationFilePath().c_str(), &para_inflation);
        inflation_radius = para_inflation.inflation_radius();
        cost_scaling_factor = para_inflation.cost_scaling_factor();
        need_reinflation_ = false;
        SetInflationParameters(inflation_radius, cost_scaling_factor);
        is_enabled_ = true;
        inflate_unknown_ = false;
        need_reinflation_ = true;
        MatchSize();
    }

    void InflationLayer::OnFootprintChanged() {
        inscribed_radius_ = layered_costmap_->GetInscribedRadius();
        cell_inflation_radius_ = CellDistance(inflation_radius_);
        ComputeCaches();
        need_reinflation_ = true;
    }

    void InflationLayer::SetInflationParameters(double inflation_radius, double cost_scaling_factor) {
        if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius) {
            std::unique_lock<std::recursive_mutex> lock(*inflation_access_);
            inflation_radius_ = inflation_radius;
            cell_inflation_radius_ = CellDistance(inflation_radius_);
            weight_ = cost_scaling_factor;
            need_reinflation_ = true;
            ComputeCaches();
        }
    }

    void InflationLayer::MatchSize() {
        std::unique_lock<std::recursive_mutex> lock(*inflation_access_);
        Costmap2D *costmap = layered_costmap_->GetCostMap();
        resolution_ = costmap->GetResolution();
        cell_inflation_radius_ = CellDistance(inflation_radius_);
        ComputeCaches();

        unsigned int size_x = costmap->GetSizeXCell(), size_y = costmap->GetSizeYCell();
        if (seen_)
            delete[] seen_;
        seen_size_ = size_x * size_y;
        seen_ = new bool[seen_size_];
    }

    // 当膨胀半径变化时，重新计算cached_distances_、cached_costs_ 这两个预计算的表
    // cached_distances_ (膨胀半径+2)×(膨胀半径+2)大小的表，记录 sqrt(i*i+j*j)  i,j = 0:膨胀半径+1，
    //                      查询当前cell到 障碍cell的距离(单位为格)时会用到
    // cached_costs_存放 预计算的栅格地图cell值 表的大小为(膨胀半径+2)×(膨胀半径+2)
    //                      cell值由 当前cell到 障碍cell的距离决定
    void InflationLayer::ComputeCaches() {
        if (cell_inflation_radius_ == 0)
            return;

        // based on the inflation radius... compute distance and cost caches
        if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
            DeleteKernels();
            //make a 2D array
            cached_costs_ = new unsigned char *[cell_inflation_radius_ + 2];
            cached_distances_ = new double *[cell_inflation_radius_ + 2];

            for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i) {
                cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
                cached_distances_[i] = new double[cell_inflation_radius_ + 2];
                for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j) {
                    cached_distances_[i][j] = hypot(i, j);
                }
            }

            cached_cell_inflation_radius_ = cell_inflation_radius_;
        }

        for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i) {
            for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j) {
                cached_costs_[i][j] = ComputeCost(cached_distances_[i][j]);
            }
        }
    }

    // 清除cached_distances_、cached_costs_两个预计算的表
    void InflationLayer::DeleteKernels() {
        if (cached_distances_ != NULL) {
            for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i) {
                if (cached_distances_[i])
                    delete[] cached_distances_[i];
            }
            if (cached_distances_)
                delete[] cached_distances_;
            cached_distances_ = NULL;
        }

        if (cached_costs_ != NULL) {
            for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i) {
                if (cached_costs_[i])
                    delete[] cached_costs_[i];
            }
            delete[] cached_costs_;
            cached_costs_ = NULL;
        }
    }

    void InflationLayer::UpdateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                      double *min_y, double *max_x, double *max_y) {
        if (need_reinflation_) {
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            *min_x = -std::numeric_limits<float>::max();
            *min_y = -std::numeric_limits<float>::max();
            *max_x = std::numeric_limits<float>::max();
            *max_y = std::numeric_limits<float>::max();
            need_reinflation_ = false;
        } else {
            double tmp_min_x = last_min_x_;
            double tmp_min_y = last_min_y_;
            double tmp_max_x = last_max_x_;
            double tmp_max_y = last_max_y_;
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;
            *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
            *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
            *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
            *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
        }
    }

    void InflationLayer::UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
        std::unique_lock<std::recursive_mutex> lock(*inflation_access_);
        if (!is_enabled_ || (cell_inflation_radius_ == 0)) {
            ROS_ERROR("Layer is not enabled or inflation radius is zero");
            return;
        }
        unsigned char *master_array = master_grid.GetCharMap();
        unsigned int size_x = master_grid.GetSizeXCell(), size_y = master_grid.GetSizeYCell();

        if (seen_ == NULL) {
            seen_size_ = size_x * size_y;
            seen_ = new bool[seen_size_];
        } else if (seen_size_ != size_x * size_y) {
            delete[] seen_;
            seen_size_ = size_x * size_y;
            seen_ = new bool[seen_size_];
        }
        memset(seen_, false, size_x * size_y * sizeof(bool));

        // We need to include in the inflation cells outside the bounding
        // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
        // up to that distance outside the box can still influence the costs
        // stored in cells inside the box.
        // 本来这个BoundingBox就已经包含膨胀半径了，但是在更新这圈膨胀半径里的cell的时候
        // 发现这些cell可能会受到 BoundingBox 外再一圈膨胀半径里 障碍物cell的影响，
        // 所以这里相当于再给BoundingBox加一圈膨胀半径了，并更新这个新BoundingBox里的cell值
        // (真是有病～～)
        min_i -= cell_inflation_radius_;
        min_j -= cell_inflation_radius_;
        max_i += cell_inflation_radius_;
        max_j += cell_inflation_radius_;

        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(int(size_x), max_i);
        max_j = std::min(int(size_y), max_j);

        // Inflation list; we append cells to visit in a list associated with its distance to the nearest obstacle
        // We use a map<distance, list> to emulate the priority queue used before, with a notable performance boost

        // Start with lethal obstacles: by definition distance is 0.0
        std::vector<CellData> &obs_bin = inflation_cells_[0.0];
        for (int j = min_j; j < max_j; j++) {
            for (int i = min_i; i < max_i; i++) {
                int index = master_grid.GetIndex(i, j);
                unsigned char cost = master_array[index];
                if (cost == LETHAL_OBSTACLE) {
                    obs_bin.push_back(CellData(index, i, j, i, j));
                }
            }
        }

        // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
        // can overtake previously inserted but farther away cells
        std::map<double, std::vector<CellData> >::iterator bin;
        for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin) {
            for (int i = 0; i < bin->second.size(); ++i) {
                // process all cells at distance dist_bin.first
                const CellData &cell = bin->second[i];

                unsigned int index = cell.index_;

                // ignore if already visited
                if (seen_[index]) {
                    continue;
                }

                seen_[index] = true;

                unsigned int mx = cell.x_;
                unsigned int my = cell.y_;
                unsigned int sx = cell.src_x_;
                unsigned int sy = cell.src_y_;

                // assign the cost associated with the distance from an obstacle to the cell
                unsigned char cost = CostLookup(mx, my, sx, sy);
                unsigned char old_cost = master_array[index];
                if (old_cost == NO_INFORMATION
                    && (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
                    master_array[index] = cost;
                else
                    master_array[index] = std::max(old_cost, cost);

                // attempt to put the neighbors of the current cell onto the inflation list
                if (mx > 0)
                    Enqueue(index - 1, mx - 1, my, sx, sy);
                if (my > 0)
                    Enqueue(index - size_x, mx, my - 1, sx, sy);
                if (mx < size_x - 1)
                    Enqueue(index + 1, mx + 1, my, sx, sy);
                if (my < size_y - 1)
                    Enqueue(index + size_x, mx, my + 1, sx, sy);
            }
        }

        inflation_cells_.clear();
    }

    /**
     * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
     * @param  grid The costmap
     * @param  index The index of the cell
     * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
     * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
     * @param  src_x The x index of the obstacle point inflation started at
     * @param  src_y The y index of the obstacle point inflation started at
     */
    inline void InflationLayer::Enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                        unsigned int src_x, unsigned int src_y) {
        if (!seen_[index]) {
            // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
            double distance = DistanceLookup(mx, my, src_x, src_y);

            // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
            if (distance > cell_inflation_radius_)
                return;

            // push the cell data onto the inflation list and mark
            inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
        }
    }
}