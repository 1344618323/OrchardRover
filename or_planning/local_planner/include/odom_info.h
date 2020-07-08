#ifndef OR_PLANNING_LOCAL_PLANNER_ODOM_INFO_H
#define OR_PLANNING_LOCAL_PLANNER_ODOM_INFO_H

#include <string>
#include <mutex>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


namespace or_local_planner {

/**
 * @brief A class of odometry
 */
class OdomInfo {
 public:

  /**
   * Constructor
   * @param topic Topic of odom info, default is null
   */
  OdomInfo(std::string topic = "");
  ~OdomInfo() {}

  /**
   * @brief Odom's callback function
   * @param msg Odom's messages
   */
  void OdomCB(const nav_msgs::Odometry::ConstPtr& msg);

  /**
   * @brief Get velocity
   * @param vel Velocity
   */
  void GetVel(tf::Stamped<tf::Pose>& vel);

  /**
   * @brief Set Odom callback
   * @param topic topic of odom info
   */
  void SetTopic(std::string topic);

  /**
   * @brief Get odom topic
   * @return Odom topic name
   */
  std::string GetTopic() const { return topic_; }

 private:

  //! odom topic name
  std::string topic_;
  //! subscriber
  ros::Subscriber sub_;
  //! odom info
  nav_msgs::Odometry odom_;
  //! odom mutex
  std::mutex mutex_;
  //! odom frame id
  std::string frame_id_;

};

} // namespace or_local_planner
#endif //OR_PLANNING_LOCAL_PLANNER_ODOM_INFO_H
