//
// Created by cxn on 2020/7/5.
//

#ifndef OR_PLANNING_LOCAL_PLANNER_NODE_H
#define OR_PLANNING_LOCAL_PLANNER_NODE_H

#include <string>
#include <thread>
#include <memory>
#include <mutex>
#include <functional>
#include <condition_variable>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <actionlib/server/simple_action_server.h>

#include "proto_io/io.h"
#include "state/error_code.h"
#include "state/node_state.h"

#include "costmap_interface.h"
#include "or_msgs/LocalPlannerAction.h"
#include "or_msgs/TwistAccel.h"


#include "local_planner_config.pb.h"
#include "local_planner_base.h"
#include "local_visualization.h"

namespace or_local_planner {

/**
 * @brief Local planner node class
 */
        class LocalPlannerNode {
            public:
            /**
             * @brief Constructor
             */
            LocalPlannerNode();
            ~LocalPlannerNode();

            /**
             * @brief init all param
             * @return Error info
             */
            or_common::ErrorInfo Init();

            /**
             * @brief Main loop
             */
            void Loop();

            /**
             * @brief Actionlib callback function use to control loop function
             * @param command Command to control loop function
             */
            void ExcuteCB(const or_msgs::LocalPlannerGoal::ConstPtr &command);

            /**
             * @brief start local planner algorithm
             */
            void StartPlanning();

            /**
             * @brief stop local planner algorithm
             */
            void StopPlanning();

            /**
             * @brief Set local planner node state
             * @param node_state State want to set
             */
            void SetNodeState(const or_common::NodeState& node_state);

            /**
             * @brief Get local planner node state
             * @return State of local planner node
             */
            or_common::NodeState GetNodeState();

            /**
             * @brief Set Error info if error occur
             * @param error_info error info
             */
            void SetErrorInfo(const or_common::ErrorInfo error_info);

            /**
             * Get error info
             * @return Error in local planner node
             */
            or_common::ErrorInfo GetErrorInfo();

            private:

            //! ros node handle
            ros::NodeHandle local_planner_nh_;
            //! local planner algorithm thread
            std::thread local_planner_thread_;
            //! local planner node actionlib server
            actionlib::SimpleActionServer<or_msgs::LocalPlannerAction> as_;
            //! local planner algorithm parent pointer
            std::unique_ptr<LocalPlannerBase> local_planner_;
            //! node state mutex
            std::mutex node_state_mtx_;
            //! node error info mutex
            std::mutex node_error_info_mtx_;
            //! planner algorithm mutex
            std::mutex plan_mtx_;
            //! node state
            or_common::NodeState node_state_;
            //! error info
            or_common::ErrorInfo node_error_info_;
            //! local cost map
            std::shared_ptr<or_costmap::CostmapInterface> local_cost_;
            //! tf pointer
            std::shared_ptr<tf::TransformListener> tf_;
            //! initialize state
            bool initialized_;

            //! local planner algorithm which choose to run
            std::string selected_algorithm_;

            //geometry_msgs::Twist cmd_vel_;
            //! robot control velocity with accelerate
            or_msgs::TwistAccel cmd_vel_;
            //! visualization ptr
            LocalVisualizationPtr visual_;
            //! frame to visualization
            std::string visual_frame_;
            //! ros publisher
            ros::Publisher vel_pub_;
            //! When no global planner give the global plan, use local goal express robot end point
            geometry_msgs::PoseStamped local_goal_;
            //! local planner algorithm max error
            int max_error_;
            //! local planner condition variable
            std::condition_variable plan_condition_;
            //! local planner mutex
            std::mutex plan_mutex_;
            //! control frequency
            double frequency_;

        };

} // namespace or_local_planner
#endif //OR_PLANNING_LOCAL_PLANNER_NODE_H
