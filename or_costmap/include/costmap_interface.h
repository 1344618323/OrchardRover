//
// Created by cxn on 2020/7/2.
//

#ifndef OR_COSTMAP_COSTMAP_INTERFACE_H
#define OR_COSTMAP_COSTMAP_INTERFACE_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <thread>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "footprint.h"
#include "layer.h"
#include "layered_costmap.h"
#include "costmap_layer.h"
#include "static_layer.h"
#include "obstacle_layer.h"
#include "inflation_layer.h"

namespace or_costmap {

/**
 * @brief Robot pose with time stamp and global frame id.
 */
    typedef struct {
        ros::Time time;
        std::string frame_id;
        Eigen::Vector3f position;
        Eigen::Matrix3f rotation;
    } RobotPose;

/**
 * @brief class Costmap Interface for users.
 */
    class CostmapInterface {
    public:
        /**
         * @brief Constructor
         * @param map_name The costmap name
         * @param tf The tf listener
         * @param map_update_frequency The frequency to update costmap
         */
        CostmapInterface(std::string map_name, tf::TransformListener &tf, std::string config_file);

        ~CostmapInterface();

        /**
         * @brief Start the costmap processing.
         */
        void Start();

        /**
         * @breif Stop the costmap.
         */
        void Stop();

        /**
         * @breif If costmap is stoped or paused, it can be resumed.
         */
        void Resume();

        /**
         * @breif The core function updating the costmap with all layers.
         */
        void UpdateMap();

        /**
         * @brief Function to pause the costmap disabling the update.
         */
        void Pause();

        /**
         * @breif Reset costmap with every layer reset.
         */
        void ResetLayers();

        /**
         * @brief Tell whether the costmap is updated.
         * @return True if every layer is current.
         */
        bool IsCurrent() {
            return layered_costmap_->IsCurrent();
        }

        /**
         * @brief Get the robot pose in global frame.
         * @param global_pose
         * @return True if got successfully.
         */
        bool GetRobotPose(tf::Stamped<tf::Pose> &global_pose) const;

        /**
         * @brief Get the costmap.
         * @return The class containing costmap data.
         */
        Costmap2D *GetCostMap() const {
            return layered_costmap_->GetCostMap();
        }

        /**
         * @brief Get robot pose with time stamped.
         * @param global_pose
         * @return True if success.
         */
        bool GetRobotPose(geometry_msgs::PoseStamped &global_pose) const;

        /**
         * @brief Get the global map frame.
         * @return The global map frame name.
         */
        std::string GetGlobalFrameID() {
            return global_frame_;
        }

        /**
         * @brief Get the base frame.
         * @return The base frame name.
         */
        std::string GetBaseFrameID() {
            return robot_base_frame_;
        }

        /**
         * @brief Get the layered costmap
         * @return The class including layers
         */
        LayeredCostmap *GetLayeredCostmap() {
            return layered_costmap_;
        }

        /**
         * @brief Get the footprint polygon which are already padded.
         * @return The footprint polygon.
         */
        geometry_msgs::Polygon GetRobotFootprintPolygon() {
            return ToPolygon(padded_footprint_);
        }

        /**
         * @brief Get the footprint points which are already padded.
         * @return The vector of footprint point.
         */
        std::vector<geometry_msgs::Point> GetRobotFootprint() {
            return padded_footprint_;
        }

        /**
         * @brief Get the footprint which are not padded.
         * @return The vector of footprint point.
         */
        std::vector<geometry_msgs::Point> GetUnpaddedRobotFootprint() {
            return unpadded_footprint_;
        }

        /**
         * @brief Get the oriented footprint in global map.
         * @param oriented_footprint
         */
        void GetOrientedFootprint(std::vector<geometry_msgs::Point> &oriented_footprint) const;

        /**
         * @brief Set up the robot footprint.
         * @param points Input vector of points to setup the robot footprint.
         */
        void SetUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point> &points);

        /**
         * @brief Set up the robot footprint.
         * @param footprint Input po"CostmapInterface"lygon to setup the robot footprint.
         */
        void SetUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon &footprint);

        /**
         * @brief Output the original footprint.
         * @param footprint Vector of Eigen::Vector3f.
         */
        void GetFootprint(std::vector<Eigen::Vector3f> &footprint);

        /**
         * @brief Get the oriented footprint.
         * @param footprint Vector of Eigen::Vector3f
         */
        void GetOrientedFootprint(std::vector<Eigen::Vector3f> &footprint);

        /**
         * @brief Get robot pose in self-defined format.
         * @param pose Pose defined by RM team.
         * @return True if get successfully
         */
        bool GetRobotPose(RobotPose &pose);

        /**
         * @brief Get the costmap value array.
         * @return The pointer to the costmap value array.
         */
        unsigned char *GetCharMap() const;

        /**
         * @brief Get the stamped pose message.
         * @param pose_msg
         * @return PoseStamped message.
         */
        geometry_msgs::PoseStamped Pose2GlobalFrame(const geometry_msgs::PoseStamped &pose_msg);

        void ClearCostMap();

        void ClearLayer(CostmapLayer *costmap_layer_ptr, double pose_x, double pose_y);

    protected:
        void LoadParameter();

        std::vector<geometry_msgs::Point> footprint_points_;
        LayeredCostmap *layered_costmap_;
        std::string name_, config_file_, config_file_inflation_,config_file_obstacle_,config_file_static_;
        tf::TransformListener &tf_;
        std::string global_frame_, robot_base_frame_;
        double transform_tolerance_, dist_behind_robot_threshold_to_care_obstacles_;
        nav_msgs::OccupancyGrid grid_;
        char *cost_translation_table_ = NULL;

        ros::Publisher costmap_pub_;
        ros::Publisher footprint_pub_;

    private:
        void DetectMovement(const ros::TimerEvent &event);

        void MapUpdateLoop(double frequency);

        std::vector<geometry_msgs::Point> unpadded_footprint_, padded_footprint_;
        float footprint_padding_;
        bool map_update_thread_shutdown_, stop_updates_, initialized_, stopped_, robot_stopped_, get_footprint_, \
 is_track_unknown_, is_rolling_window_, has_static_layer_, has_obstacle_layer_;
        double map_update_frequency_, map_width_, map_height_, map_origin_x_, map_origin_y_, map_resolution_;
        std::thread *map_update_thread_;
        ros::Timer timer_;
        ros::Time last_publish_;
        tf::Stamped<tf::Pose> old_pose_;
    };
}
#endif //OR_COSTMAP_COSTMAP_INTERFACE_H
