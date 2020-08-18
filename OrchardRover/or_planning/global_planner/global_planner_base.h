//
// Created by cxn on 2020/7/3.
//

#ifndef OR_PLANNING_GLOBAL_PLANNER_BASE_H
#define OR_PLANNING_GLOBAL_PLANNER_BASE_H

#include "state/error_code.h"
#include "costmap_interface.h"

namespace or_global_planner{

    class GlobalPlannerBase {
    public:
        typedef std::shared_ptr<or_costmap::CostmapInterface> CostmapPtr;

        GlobalPlannerBase(CostmapPtr costmap_ptr)
                : costmap_ptr_(costmap_ptr) {
        };
        virtual ~GlobalPlannerBase() = default;

        virtual or_common::ErrorInfo Plan(const geometry_msgs::PoseStamped &start,
                                               const geometry_msgs::PoseStamped &goal,
                                               std::vector<geometry_msgs::PoseStamped> &path) = 0;

    protected:
        CostmapPtr costmap_ptr_;
    };

} //namespace or_global_planner

#endif //OR_PLANNING_GLOBAL_PLANNER_BASE_H
