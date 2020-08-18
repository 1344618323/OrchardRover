#include "rigid_transform.h"

namespace optimized_slam
{
  namespace transform
  {
    Eigen::Quaterniond RollPitchYaw(const double roll, const double pitch,
                                    const double yaw)
    {
      const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
      const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
      const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
      return yaw_angle * pitch_angle * roll_angle;
    }
  } // namespace transform
} // namespace optimized_slam