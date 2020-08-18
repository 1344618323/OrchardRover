#ifndef OR_EXECUTOR_GOAL_BEHAVIOR_H
#define OR_EXECUTOR_GOAL_BEHAVIOR_H

#include "chassis_executor.h"

namespace or_executor {
    class GoalBehavior {
    public:
        GoalBehavior(ChassisExecutor *chassis_executor) : chassis_executor_(chassis_executor) {}

        void Run(const geometry_msgs::PoseStamped &goal) {
            chassis_executor_->Execute(goal);
        }

        void Cancel() {
            chassis_executor_->Cancel();
        }

        ~GoalBehavior() = default;

    private:
        //! executor
        ChassisExecutor *const chassis_executor_;

        //! planning goal
        geometry_msgs::PoseStamped planning_goal_;
    };
} // namespace or_executor

#endif //or_executor_GOAL_BEHAVIOR_H
