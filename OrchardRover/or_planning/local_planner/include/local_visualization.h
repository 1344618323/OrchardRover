#ifndef OR_PLANNING_LOCAL_PLANNER_LOCAL_VISUALIZATION_H
#define OR_PLANNING_LOCAL_PLANNER_LOCAL_VISUALIZATION_H

#include <iterator>
#include <memory>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include "teb_vertex_console.h"


namespace or_local_planner {

/**
 * @brief Class use to visualize local planner algorithm's trajectory
 */
    class LocalVisualization {
    public:
        /**
         * @brief Constructor
         */
        LocalVisualization();

        /**
         * @brief Constructor initialize this class
         * @param nh Ros node handle
         * @param visualize_frame Visualize frame
         */
        LocalVisualization(ros::NodeHandle &nh, const std::string &visualize_frame);

        /**
         * @brief Initialize visualize frame and ros param
         * @param nh ros node handle
         * @param visualize_frame Visualize frame
         */
        void Initialization(ros::NodeHandle &nh, const std::string &visualize_frame);

        /**
         * @brief publish trajectory
         * @param vertex_console Robot trajectory from local planner algorithm
         */
        void PublishLocalPlan(const TebVertexConsole &vertex_console) const;

    protected:

        //! trajectory publisher
        ros::Publisher local_planner_;
        //! trajectory pose publisher
        ros::Publisher pose_pub_;

        //! visualize frame
        std::string visual_frame_ = "map";

        //! initialize state
        bool initialized_;

    };

    typedef std::shared_ptr<const LocalVisualization> LocalVisualizationPtr;

} // namespace or_local_planner

#endif // or_PLANNING_LOCAL_PLANNER_LOCAL_VISUALIZATION_H