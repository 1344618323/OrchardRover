#ifndef OR_PLANNING_LOCAL_PLANNER_TEB_PENALTIES_H
#define OR_PLANNING_LOCAL_PLANNER_TEB_PENALTIES_H

#include <cmath>
#include <Eigen/Core>
#include <g2o/stuff/misc.h>

namespace or_local_planner {

    //<-a+epsilon: 惩罚 (-a + epsilon) -var
    //[-a+epsilon,a-epsilon]: 无惩罚，0
    //>a-epsilon: 惩罚 var - (a - epsilon)
    inline double PenaltyBoundToInterval(const double &var, const double &a, const double &epsilon) {
        if (var < -a + epsilon) {
            return (-var - (a - epsilon));
        }
        if (var <= a - epsilon) {
            return 0.;
        } else {
            return (var - (a - epsilon));
        }
    }

    // <a+epsilon: 惩罚 (a + epsilon) -var
    // [a+epsilon,b-epsilon]:无惩罚,0
    // >b-epsilon: 惩罚 var - (b - epsilon)
    inline double PenaltyBoundToInterval(const double &var, const double &a, const double &b, const double &epsilon) {
        if (var < a + epsilon) {
            return (-var + (a + epsilon));
        }
        if (var <= b - epsilon) {
            return 0.;
        } else {
            return (var - (b - epsilon));
        }
    }

    // >=a+epsilon: 无惩罚，0
    // < a+epsilon: 惩罚，(a+epsilon)-var
    inline double PenaltyBoundFromBelow(const double &var, const double &a, const double &epsilon) {
        if (var >= a + epsilon) {
            return 0.;
        } else {
            return (-var + (a + epsilon));
        }
    }

    inline double PenaltyBoundToIntervalDerivative(const double &var, const double &a, const double &epsilon) {
        if (var < -a + epsilon) {
            return -1;
        }
        if (var <= a - epsilon) {
            return 0.;
        } else {
            return 1;
        }
    }

    inline double PenaltyBoundToIntervalDerivative(const double &var,
                                                   const double &a,
                                                   const double &b,
                                                   const double &epsilon) {
        if (var < a + epsilon) {
            return -1;
        }
        if (var <= b - epsilon) {
            return 0.;
        } else {
            return 1;
        }
    }

    inline double PenaltyBoundFromBelowDerivative(const double &var, const double &a, const double &epsilon) {
        if (var >= a + epsilon) {
            return 0.;
        } else {
            return -1;
        }
    }

} // namespace or_local_planner
#endif // oR_PLANNING_LOCAL_PLANNER_TEB_PENALTIES_H
