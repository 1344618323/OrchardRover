#ifndef OR_PLANNING_LOCAL_PLANNER_TEB_H
#define OR_PLANNING_LOCAL_PLANNER_TEB_H

#include <mutex>

#include "proto_io/io.h"
#include "state/error_code.h"

#include "costmap_interface.h"

#include "odom_info.h"

#include "local_planner_base.h"

#include "teb_optimal.h"

namespace or_local_planner {

/**
 * @brief See local_planner_base.h
 */
class TebLocalPlanner : public LocalPlannerBase {
 public:
  TebLocalPlanner();
  ~TebLocalPlanner();
  or_common::ErrorInfo ComputeVelocityCommands(or_msgs::TwistAccel &cmd_vel) override;
  bool IsGoalReached () override;
  or_common::ErrorInfo Initialize (std::shared_ptr<or_costmap::CostmapInterface> local_cost,
                   std::shared_ptr<tf::TransformListener> tf, LocalVisualizationPtr visual) override;
  bool SetPlan(const nav_msgs::Path& plan, const geometry_msgs::PoseStamped& goal) override ;
  bool GetPlan(const nav_msgs::Path& plan);
  bool SetPlanOrientation();
  void RegisterErrorCallBack(ErrorInfoCallback error_callback) override;

  bool PruneGlobalPlan();

  bool TransformGlobalPlan(int *current_goal_idx = NULL);

  double EstimateLocalGoalOrientation(const DataBase& local_goal,
                                      int current_goal_idx, int moving_average_length=3) const;

  void UpdateViaPointsContainer();

  void SaturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y,
                        double max_vel_theta, double max_vel_x_backwards) const;

  void UpdateObstacleWithCostmap(Eigen::Vector2d local_goal);
  void UpdateRobotPose();
  void UpdateRobotVel();
  void UpdateGlobalToPlanTranform();
  bool CutAndTransformGlobalPlan(int *current_goal_idx = NULL);

  double ConvertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius = 0) const;

  //! Tf listener
  std::weak_ptr<tf::TransformListener> tf_;
  //! Local cost map
  std::weak_ptr<or_costmap::CostmapInterface> local_cost_;

  //! Local planner frame(local planner will do optimal in this frame), different with global planner frame
  std::string global_frame_;
  //! Local planner costmap 2d
  or_costmap::Costmap2D *costmap_;
  //! Robot footprint cost
  std::shared_ptr<or_local_planner::RobotPositionCost> robot_cost_;
  //! Optimal based algorithm ptr
  TebOptimalPtr optimal_;
  //! Obstacle ptr
  std::vector<ObstaclePtr> obst_vector_;
  //! Must via point
  ViaPointContainer via_points_;
  //! Robot footprint
  std::vector<Eigen::Vector2d> robot_footprint_;
  //! Robot inscribed radius
  double robot_inscribed_radius_;
  //! Robot circumscribed radius
  double robot_circumscribed_radius;
  //! Robot odom info
  // OdomInfo odom_info_;
  //! Global planner's solve
  nav_msgs::Path global_plan_, temp_plan_;
  //! Last velocity
  or_msgs::TwistAccel last_cmd_;
  //! Robot current velocity
  geometry_msgs::Twist robot_current_vel_;
  //! Robot current pose
  DataBase robot_pose_;
  //! Robot current pose
  tf::Stamped<tf::Pose> robot_tf_pose_;//机器人在odom系位姿
  //! Robot goal
  DataBase robot_goal_;
  //! Visualize ptr use to visualize trajectory after optimize
  LocalVisualizationPtr visual_;
  //! Tf transform from global planner frame to optimal frame
  tf::StampedTransform plan_to_global_transform_;
  //! Way point after tf transform
  std::vector<DataBase> transformed_plan_;//待优化路径
  //! When no global planner give the global plan, use local goal express robot end point
  tf::Stamped<tf::Pose> local_goal_;
  //! Error info when running teb local planner algorithm
  or_common::ErrorInfo teb_error_info_;
  //! Call back function use to return error info
  ErrorInfoCallback error_callback_;
  //! Time begin when robot oscillation at a position
  std::chrono::system_clock::time_point oscillation_;
  //! Time allow robot oscillation at a position
  double oscillation_time_;
  //! Robot last position
  DataBase last_robot_pose_;
  //! Plan mutex
  std::mutex plan_mutex_;

  //! Optimal param
  Config param_config_;
  bool  free_goal_vel_;
  bool  global_plan_overwrite_orientation_;
  float cut_lookahead_dist_;
  long  fesiable_step_look_ahead_;
  float max_vel_x_;
  float max_vel_y_;
  float max_vel_theta_;
  float max_vel_x_backwards;
  float xy_goal_tolerance_;
  float yaw_goal_tolerance_;
  float osbtacle_behind_robot_dist_;


 protected:
  //! Check if the algorithm is initialized
  bool is_initialized_ = false;
};
} // namespace or_local_planner




#endif 