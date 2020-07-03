//
// Created by cxn on 2020/7/1.
//

#ifndef ROBORTS_COSTMAP_LAYERED_COSTMAP_H
#define ROBORTS_COSTMAP_LAYERED_COSTMAP_H

#include <geometry_msgs/Point.h>
#include "costmap_2d.h"
#include "footprint.h"
#include "layer.h"

namespace or_costmap {
    /*
     * LayeredCostmap就是 master_costmap的类型，由多layer叠加而成
     * Costmap2D就是栅格地图(grid)，用于存储val
     * Layer是层的抽象类，包括了updateBounds、updateCosts等常用方法
     * CostmapLayer:继承了layer与Costmap2D：即有独立存储栅格地图空间的Layer
     * ObstacleLayer StaticLayer都是继承自CostmapLayer
     * 而InflationLayer继承自Layer(其不需要grid)
     * LayeredCostmap包含了多个Layer子类的对象，通过调用这些层的updateBounds、updateCosts等函数，
     *               更新LayeredCostmap包含的Costmap2D对象(grid)
     */

    static const unsigned char NO_INFORMATION = 255;
    static const unsigned char LETHAL_OBSTACLE = 254;
    static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
    static const unsigned char FREE_SPACE = 0;

    class Layer;

    class LayeredCostmap {
    public:
        LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);

        ~LayeredCostmap();

        void UpdateMap(double robot_x, double robot_y, double robot_yaw);

        const std::vector<geometry_msgs::Point> &GetFootprint() {
            return footprint_;
        }

        std::string GetGlobalFrameID() const {
            return global_frame_id_;
        }

        bool IsRollingWindow() const {
            return is_rolling_window_;
        }

        bool IsSizeLocked() const {
            return is_size_locked_;
        }

        bool IsTrackingUnknown() const {
            return costmap_.GetDefaultValue() == NO_INFORMATION;
        }

        void ResizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y,
                       bool size_locked = false);

        Costmap2D *GetCostMap() {
            return &costmap_;
        }

        void GetUpdatedBounds(double &minx, double &miny, double &maxx, double &maxy) {
            minx = minx_;
            miny = miny_;
            maxx = maxx_;
            maxy = maxy_;
        }

        bool IsCurrent();

        bool IsRolling() {
            return is_rolling_window_;
        }

        std::vector<Layer *> *GetPlugins() {
            return &plugins_;
        }

        void AddPlugin(Layer *plugin) {
            plugins_.push_back(plugin);
        }

        bool IsSizeLocked() {
            return is_size_locked_;
        }

        void GetBounds(unsigned int *x0, unsigned int *xn, unsigned int *y0, unsigned int *yn) {
            *x0 = bx0_;
            *xn = bxn_;
            *y0 = by0_;
            *yn = byn_;
        }

        bool IsInitialized() {
            return is_initialized_;
        }

        /** @brief Updates the stored footprint, updates the circumscribed
          * and inscribed radii, and calls onFootprintChanged() in all
          * layers. */
        void SetFootprint(const std::vector<geometry_msgs::Point> &footprint_spec);

        /** @brief The radius of a circle centered at the origin of the
         * robot which just surrounds all points on the robot's
         * footprint.
         *
         * This is updated by setFootprint(). */
        double GetCircumscribedRadius() { return circumscribed_radius_; }

        /** @brief The radius of a circle centered at the origin of the
         * robot which is just within all points and edges of the robot's
         * footprint.
         *
         * This is updated by setFootprint(). */
        double GetInscribedRadius() { return inscribed_radius_; }

        /**
         * @brief Set the inflation layer prototxt path for the inflaiton layer to read in
         * @param path The file path.
         */
        void SetInflationFilePath(const std::string &path) {
            inflation_file_path_ = path;
        }

        std::string GetInflationFilePath() const {
            return inflation_file_path_;
        }

        void SetObstacleFilePath(const std::string &path) {
            obstacle_file_path_ = path;
        }

        std::string GetObstacleFilePath() const {
            return obstacle_file_path_;
        }

        void SetStaticFilePath(const std::string &path) {
            static_file_path_ = path;
        }

        std::string GetStaticFilePath() const {
            return static_file_path_;
        }

    private:
        std::string global_frame_id_, inflation_file_path_,obstacle_file_path_,static_file_path_;
        std::vector<geometry_msgs::Point> footprint_;
        Costmap2D costmap_;
        bool is_rolling_window_, is_size_locked_, is_initialized_, is_current_;
        double minx_, miny_, maxx_, maxy_, circumscribed_radius_, inscribed_radius_;
        unsigned int bx0_, bxn_, by0_, byn_;
        std::vector<Layer *> plugins_;
    };
}
#endif //ROBORTS_COSTMAP_LAYERED_COSTMAP_H
