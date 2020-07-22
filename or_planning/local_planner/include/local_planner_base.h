//
// Created by cxn on 2020/7/5.
//

#ifndef OR_PLANNING_LOCAL_PLANNER_BASE_H
#define OR_PLANNING_LOCAL_PLANNER_BASE_H

#include <functional>
#include <memory>
#include <tf/transform_listener.h>
#include "state/error_code.h"
#include "costmap_interface.h"
#include "or_msgs/TwistAccel.h"
#include "local_visualization.h"

/**
 * @brief parent class of all local planner algorithms
 */
namespace or_local_planner {
    //! Error info callback function
    typedef std::function<void(const or_common::ErrorInfo &)> ErrorInfoCallback;

    class LocalPlannerBase {
    public:
        /**
         * @brief Virtual function calculate robot velocity should be implemented by derived class
         * @param cmd_vel Velocity use to control robot
         * @return Error info
         */
        virtual or_common::ErrorInfo ComputeVelocityCommands(or_msgs::TwistAccel &cmd_vel) = 0;

        /**
         * @brief Virtual function judge if robot reach the goal should be implemented by derived class
         * @return If true reached the goal, else not
         */
        virtual bool IsGoalReached() = 0;

        /**
         * @brief Virtual function initialize local planner algorithm should be implemented by derived class
         * @param local_cost Local cost map
         * @param tf Tf listener
         * @param visual Visualize pointer
         * @return Error info
         */
        virtual or_common::ErrorInfo Initialize(std::shared_ptr<or_costmap::CostmapInterface> local_cost,
                                                std::shared_ptr<tf::TransformListener> tf,
                                                LocalVisualizationPtr visual) = 0;

        /**
         * @brief Virtual function Set global plan's result to local planner or set a goal to local planner
         * should be implement by derived class
         * @param plan Result of global planner
         * @param goal Goal of local planner
         * @return If true success, else fail
         */
        virtual bool SetPlan(const nav_msgs::Path &plan, const geometry_msgs::PoseStamped &goal) = 0;

        /**
         * @brief Virtual function Register error callback function should be implemented by derived class
         * @param error_callback Callback function
         */
        virtual void RegisterErrorCallBack(ErrorInfoCallback error_callback) = 0;

        virtual ~LocalPlannerBase() {}

    protected:
        LocalPlannerBase() {}
    };

    typedef std::shared_ptr<LocalPlannerBase> LocalPlannerPtr;

} // namespace or_local_planner
#endif //OR_PLANNING_LOCAL_PLANNER_BASE_H
