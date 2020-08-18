#ifndef OR_PLANNING_LOCAL_PLANNER_TEB_OPTIMAL_H
#define OR_PLANNING_LOCAL_PLANNER_TEB_OPTIMAL_H

#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <boost/thread/once.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include "state/error_code.h"

#include "robot_position_cost.h"
#include "obstacle.h"
#include "robot_footprint_model.h"
#include "utility_tool.h"
#include "local_visualization.h"
#include "robot_position_cost.h"

#include "teb_vertex_console.h"
#include "timed_elastic_band.pb.h"

#include "g2o_type/teb_acceleration_edge.h"
#include "g2o_type/teb_kinematics_edge.h"
#include "g2o_type/teb_obstacle_edge.h"
#include "g2o_type/teb_prefer_rotdir_edge.h"
#include "g2o_type/teb_time_optimal_edge.h"
#include "g2o_type/teb_velocity_edge.h"
#include "g2o_type/teb_via_point_edge.h"

namespace or_local_planner {

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > TebBlockSolver;
    typedef g2o::LinearSolverCSparse<TebBlockSolver::PoseMatrixType> TebLinearSolver;
    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > ViaPointContainer;


    class TebOptimal {
    public:
        /**
         * @brief Constructor
         */
        TebOptimal();

        TebOptimal(const Config &config_param, ObstContainer *obstacles = NULL,
                   RobotFootprintModelPtr robot_model = std::make_shared<PointRobotFootprint>(),
                   LocalVisualizationPtr visual = LocalVisualizationPtr(), const ViaPointContainer *via_points = NULL);

        ~TebOptimal() {

        }

        void initialize(const Config &config_param, ObstContainer *obstacles = NULL,
                        RobotFootprintModelPtr robot_model = std::make_shared<PointRobotFootprint>(),
                        LocalVisualizationPtr visual = LocalVisualizationPtr(),
                        const ViaPointContainer *via_points = NULL);

        /**
         * @brief Entry function of optimal based planner algorithm
         * @param initial_plan Input and output, way point want to optimal
         * @param start_vel Start velocity of robot
         * @param free_goal_vel If true set the goal velocity to zero
         * @param micro_control If true when the goal behind the robot, robot will have negative x velocity
         * @return True if success, otherwise false
         */
        bool Optimal(std::vector<DataBase> &initial_plan, const geometry_msgs::Twist *start_vel = NULL,
                     bool free_goal_vel = false, bool micro_control = false);

        /**
         * @brief Entry function of optimal based planner algorithm
         * @param start Start position (maybe robot current position)
         * @param goal Goal position (where robot want to go)
         * @param start_vel Robot current velocity
         * @param free_goal_vel If true set the goal velocity to zero
         * @param micro_control If true when the goal behind the robot, robot will have negative x velocity
         * @return True if success, otherwise false
         */
        bool Optimal(const DataBase &start, const DataBase &goal, const geometry_msgs::Twist *start_vel = NULL,
                     bool free_goal_vel = false, bool micro_control = false);

        /**
         * @brief Get next control velocity
         * @param error_info Error info when get the robot velocity
         * @param vx X component velocity
         * @param vy Y component velocity
         * @param omega Angular velocity
         * @param acc_x X component acceleration
         * @param acc_y Y component acceleration
         * @param acc_omega Angular acceleration
         * @return True if success, otherwise false
         */
        bool GetVelocity(or_common::ErrorInfo &error_info, double &vx, double &vy, double &omega,
                         double &acc_x, double &acc_y, double &acc_omega) const;


        /**
         * @brief Visualize trajectory and pose after optimize
         */
        void Visualize();


        /**
         * @brief Clear all vertices and edges
         */
        void ClearPlanner() {
            ClearGraph();
        }

        static void RegisterG2OTypes();

        void ComputeCurrentCost(double obst_cost_scale = 1.0, double viapoint_cost_scale = 1.0,
                                bool alternative_time_cost = false);

        /**
         * @brief Compute current graph cost(errors)
         * @param cost Input and output, total errors of all edges
         * @param obst_cost_scale Obstacle's scale number
         * @param alternative_time_cost True use trajectory time diff add to errors
         */
        void ComputeCurrentCost(std::vector<double> &cost, double obst_cost_scale = 1.0,
                                double viapoint_cost_scale = 1.0, bool alternative_time_cost = false) {
            ComputeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
            cost.push_back(GetCurrentCost());
        }

        double GetCurrentCost() const {
            return cost_;
        }


        /**
         * @brief Judge if the trajectory after optimize is feasible
         * @param error_info Error info about trajectory feasible judge
         * @param position_cost Cost map which to calculate the point cost
         * @param footprint_spec Robot vertices
         * @param inscribed_radius Robot inscribed radius
         * @param circumscribed_radius Robot circumscribed radius
         * @param look_ahead_idx How many point want to check
         * @return True if feasible, otherwise is false
         */
        bool IsTrajectoryFeasible(or_common::ErrorInfo &error_info, RobotPositionCost *position_cost,
                                  const std::vector<Eigen::Vector2d> &footprint_spec,
                                  double inscribed_radius = 0.0, double circumscribed_radius = 0.0,
                                  int look_ahead_idx = -1);


    private:
        void SetVelocityStart(const geometry_msgs::Twist &vel_start);

        void SetVelocityEnd(const geometry_msgs::Twist &vel_end);

        void SetVelocityGoalFree() {
            vel_end_.first = false;
        }

        inline void ExtractVelocity(const DataBase &pose1, const DataBase &pose2, double dt,
                                    double &vx, double &vy, double &omega) const;

        bool OptimizeTeb(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards = false,
                         double obst_cost_scale = 1.0, double viapoint_cost_scale = 1.0,
                         bool alternative_time_cost = false);

        bool BuildGraph(double weight_multiplier = 1.0);

        bool OptimizeGraph(int no_iterations, bool clear_after = true);

        void ClearGraph();

        void AddTebVertices();

        void AddVelocityEdges();

        void AddAccelerationEdges();

        void AddTimeOptimalEdges();

        void AddObstacleEdges(double weight_multiplier = 1.0);

        void AddObstacleLegacyEdges(double weight_multiplier = 1.0);

        void AddViaPointsEdges();

        void AddPreferRotDirEdges();

        void AddKinematicsDiffDriveEdges();

        void AddKinematicsCarlikeEdges();

        void AddDynamicObstaclesEdges();


        std::shared_ptr<g2o::SparseOptimizer> InitOptimizer();

        ObstContainer *obstacles_;
        const ViaPointContainer *via_points_;
        double cost_;
        RotType prefer_rotdir_;
        LocalVisualizationPtr visualization_;

        RobotFootprintModelPtr robot_model_;
        TebVertexConsole vertex_console_;
        std::shared_ptr<g2o::SparseOptimizer> optimizer_;
        std::pair<bool, geometry_msgs::Twist> vel_start_;
        std::pair<bool, geometry_msgs::Twist> vel_end_;

        Robot robot_info_;
        Config param_config_;
        Obstacles obstacles_info_;
        Trajectory trajectory_info_;
        Optimization optimization_info_;

        bool initialized_;
        bool optimized_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };


    typedef std::shared_ptr<TebOptimal> TebOptimalPtr;

} // namespace or_local_planner

#endif  // OR_PLANNING_LOCAL_PLANNER_TEB_OPTIMAL_H