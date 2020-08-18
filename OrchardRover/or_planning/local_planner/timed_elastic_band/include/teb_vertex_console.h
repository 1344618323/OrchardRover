//
// Created by cxn on 2020/7/6.
//

#ifndef TIMED_ELASTIC_BAND_TEB_VERTEX_CONSOLE_H
#define TIMED_ELASTIC_BAND_TEB_VERTEX_CONSOLE_H

#include <iterator>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>

#include "obstacle.h"
#include "data_base.h"
#include "data_converter.h"
#include "distance_calculation.h"

#include "g2o_type/teb_vertex_pose.h"
#include "g2o_type/teb_vertex_timediff.h"

namespace or_local_planner
{

    typedef std::vector<TebVertexPose *> PoseSequence;

    typedef std::vector<TebVertexTimeDiff *> TimeDiffSequence;

    /**
     * @brief graph vertices
     */
    class TebVertexConsole
    {
    public:
        TebVertexConsole();

        virtual ~TebVertexConsole();

        PoseSequence &Poses() { return pose_vec_; };

        const PoseSequence &Poses() const
        {
            return pose_vec_;
        };

        TimeDiffSequence &TimeDiffs()
        {
            return timediff_vec_;
        };

        const TimeDiffSequence &TimeDiffs() const
        {
            return timediff_vec_;
        };

        double &TimeDiff(int index)
        {
            return timediff_vec_.at(index)->GetDiffTime();
        }

        const double &TimeDiff(int index) const
        {
            return timediff_vec_.at(index)->GetDiffTime();
        }

        DataBase &Pose(int index)
        {
            return pose_vec_.at(index)->GetPose();
        }

        const DataBase &Pose(int index) const
        {
            return pose_vec_.at(index)->GetPose();
        }

        DataBase &BackPose()
        {
            return pose_vec_.back()->GetPose();
        }

        const DataBase &BackPose() const
        {
            return pose_vec_.back()->GetPose();
        }

        double &BackTimeDiff()
        {
            return timediff_vec_.back()->GetDiffTime();
        }

        const double &BackTimeDiff() const
        {
            return timediff_vec_.back()->GetDiffTime();
        }

        TebVertexPose *PoseVertex(int index)
        {
            return pose_vec_.at(index);
        }

        TebVertexTimeDiff *TimeDiffVertex(int index)
        {
            return timediff_vec_.at(index);
        }

        void AddPose(const DataBase &pose, bool fixed = false);

        void AddPose(const Eigen::Ref<const Eigen::Vector2d> &position, double theta, bool fixed = false);

        void AddTimeDiff(double dt, bool fixed = false);

        void AddPoseAndTimeDiff(const DataBase &pose, double dt);

        void AddPoseAndTimeDiff(const Eigen::Ref<const Eigen::Vector2d> &position, double theta, double dt);

        void InsertPose(int index, const DataBase &pose);

        void InsertPose(int index, const Eigen::Ref<const Eigen::Vector2d> &position, double theta);

        void InsertTimeDiff(int index, double dt);

        void DeletePose(int index);

        void DeletePose(int index, int number);

        void DeleteTimeDiff(int index);

        void DeleteTimeDiff(int index, int number);

        bool InitTEBtoGoal(const DataBase &start,
                           const DataBase &goal,
                           double diststep = 0,
                           double timestep = 1,
                           int min_samples = 3,
                           bool guess_backwards_motion = false);

        template <typename BidirIter, typename Fun>
        bool InitTEBtoGoal(BidirIter path_start,
                           BidirIter path_end,
                           Fun fun_position,
                           double max_vel_x,
                           double max_vel_theta,
                           boost::optional<double> max_acc_x,
                           boost::optional<double> max_acc_theta,
                           boost::optional<double> start_orientation,
                           boost::optional<double> goal_orientation,
                           int min_samples = 3,
                           bool guess_backwards_motion = false);

        bool InitTEBtoGoal(std::vector<DataBase> &plan,
                           double dt,
                           bool estimate_orient = false,
                           int min_samples = 3,
                           bool guess_backwards_motion = false,
                           bool micro_control = false);

        void UpdateAndPruneTEB(boost::optional<const DataBase &> new_start,
                               boost::optional<const DataBase &> new_goal,
                               int min_samples = 3);

        void AutoResize(double dt_ref, double dt_hysteresis, int min_samples = 3, int max_samples = 1000);

        void SetPoseVertexFixed(int index, bool status);

        void SetTimeDiffVertexFixed(int index, bool status);

        void ClearAllVertex();

        int FindClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d> &ref_point,
                                      double *distance = NULL,
                                      int begin_idx = 0) const;

        int FindClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d> &ref_line_start,
                                      const Eigen::Ref<const Eigen::Vector2d> &ref_line_end,
                                      double *distance = NULL) const;

        int FindClosestTrajectoryPose(const Point2dContainer &vertices, double *distance = NULL) const;

        int FindClosestTrajectoryPose(const Obstacle &obstacle, double *distance = NULL) const;

        int SizePoses() const
        {
            return (int)pose_vec_.size();
        }

        int SizeTimeDiffs() const
        {
            return (int)timediff_vec_.size();
        }

        bool IsInit() const
        {
            return !timediff_vec_.empty() && !pose_vec_.empty();
        }

        double GetSumOfAllTimeDiffs() const;

        double GetAccumulatedDistance() const;

        bool DetectDetoursBackwards(double threshold = 0) const;

        bool IsTrajectoryInsideRegion(double radius, double max_dist_behind_robot = -1, int skip_poses = 0);

    protected:
        PoseSequence pose_vec_;
        TimeDiffSequence timediff_vec_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace or_local_planner
#endif //TIMED_ELASTIC_BAND_TEB_VERTEX_CONSOLE_H
