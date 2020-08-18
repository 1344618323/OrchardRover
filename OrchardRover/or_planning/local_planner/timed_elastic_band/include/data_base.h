//
// Created by cxn on 2020/7/6.
//

#ifndef OR_PLANNING_DATA_BASE_H
#define OR_PLANNING_DATA_BASE_H

#include <iostream>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <g2o/stuff/misc.h>

namespace or_local_planner {
/**
 * @brief Class of SE(2) data base which is used by teb_optimal
 */
class DataBase {
 public:
  /**
   * @brief default Constructor all elements will be set to zero
   */
  DataBase () {
    SetZero();
  }

  /**
   * @brief Constructor with param
   * @param position Position in optimal frame
   * @param theta Orientation in optimal frame
   */
  DataBase (const Eigen::Ref<const Eigen::Vector2d>& position, double theta) {
    position_ = position;
    theta_ = theta;
  }

  /**
   * @brief Destructor
   */
  ~DataBase() {}

  /**
   * @brief Set position param
   * @param position The new value of position_
   */
  void SetPosition(Eigen::Vector2d position) {
    position_ = position;
  }

  /**
 * @brief Set orientation
 * @param theta The new value of theta_
 */
  void SetTheta(double theta) {
    theta_ = theta;
  }

  /**
   * @brief Get positon_
   * @return The reference of position_
   */
  Eigen::Vector2d& GetPosition () {
    return  position_;
  }

  /**
   * @brief Get positon_
   * @return The reference of position_
   */
  const Eigen::Vector2d& GetPosition () const {
    return position_;
  }

  /**
   * @brief Get orientation
   * @return The reference of theta_
   */
  double& GetTheta() {
    return theta_;
  }

  /**
   * @brief Get orientation
   * @return The reference of theta_
   */
  const double& GetTheta() const {
    return theta_;
  }

  /**
   * @brief Add operation with array
   * @param pose_array a array include x, y, theta.
   */
  void Plus (const double* pose_array) {
    position_.coeffRef(0) += pose_array[0];
    position_.coeffRef(1) += pose_array[1];
    theta_ = g2o::normalize_theta( theta_ + pose_array[2] );
  }

  /**
   * @brief Average operation in two pose.
   * @param pose1 The first pose
   * @param pose2 The second pose
   */
  void AverageInPlace (const DataBase& pose1, const DataBase& pose2) {
    position_ = (pose1.position_ + pose2.position_)/2;
    theta_ = g2o::average_angle(pose1.theta_, pose2.theta_);
  }

  /**
   * @brief Rotate operation
   * @param angle Rotate angle
   * @param adjust_theta If true will normalize the theta_ with angle, otherewise will keep the orientation in the new position
   */
  void RotateGlobal (double angle, bool adjust_theta=true) {
    double new_x = std::cos(angle)*position_.x() - std::sin(angle)*position_.y();
    double new_y = std::sin(angle)*position_.x() + std::cos(angle)*position_.y();
    position_.x() = new_x;
    position_.y() = new_y;
    if (adjust_theta)
      theta_ = g2o::normalize_theta(theta_+angle);
  }

  /**
   * @brief Obtain the orientation vector
   * @return A orientation vector in form of Eigen::Vector2d
   */
  Eigen::Vector2d OrientationUnitVec() const {
    return Eigen::Vector2d(std::cos(theta_), std::sin(theta_));
  }

  /**
   * @brief Set all elements to zero
   */
  void SetZero() {
    position_.setZero();
    theta_ = 0;
  }

  /**
   * @brief Overload '=' operator
   * @param rhs The right value
   * @return A reference of the new value
   */
  DataBase& operator= ( const DataBase& rhs ) {
    if (&rhs != this) {
      position_ = rhs.position_;
      theta_ = rhs.theta_;
    }
    return *this;
  }

  /**
   * @brief Overload '+=' operator
   * @param rhs The right value
   * @return A reference of the new value
   */
  DataBase& operator+= (const DataBase& rhs) {
    position_ += rhs.position_;
    theta_ = g2o::normalize_theta(theta_ + rhs.theta_);
    return *this;
  }

  /**
   * @brief Overload '+' operator
   * @param lhs The left value
   * @param rhs The right value
   * @return New value after '+' operation
   */
  friend DataBase operator+ (DataBase lhs, const DataBase& rhs) {
    return lhs += rhs;
  }

  /**
   * @brief Overload '-=' operator
   * @param rhs The right value
   * @return A reference of new value
   */
  DataBase& operator-= (const DataBase& rhs) {
    theta_ -= rhs.theta_;
    theta_ = g2o::normalize_theta(theta_ - rhs.theta_);
    return *this;
  }

  /**
   * @brief Overload '-' operator
   * @param lhs The left value
   * @param rhs The right value
   * @return New value after '-' operation
   */
  friend DataBase operator- (DataBase lhs, const DataBase& rhs) {
    return lhs -= rhs;
  }

  /**
   * @brief Overload '*' operator
   * @param pose Data want to scale
   * @param scalar Scaling factor
   * @return Data after scale
   */
  friend DataBase operator* (DataBase pose, double scalar) {
    pose.position_ *= scalar;
    pose.theta_ *= scalar;
    return pose;
  }



 private:
  //! x, y data
  Eigen::Vector2d position_;
  //! orientation
  double theta_;

 public:
  //! Data alignment
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; // class TebDataBase

} // namespace or_local_planner
#endif //OR_PLANNING_DATA_BASE_H
