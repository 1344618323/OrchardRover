//
// Created by cxn on 2020/7/1.
//

#ifndef OR_COSTMAP_COSTMAP_MATH_H
#define OR_COSTMAP_COSTMAP_MATH_H

#include <math.h>
#include <algorithm>
#include <vector>
#include <geometry_msgs/Point.h>

namespace or_costmap {
    /** @brief Return -1 if x < 0, +1 otherwise. */
    inline double sign(double x) {
        return x < 0.0 ? -1.0 : 1.0;
    }

    /** @brief Same as sign(x) but returns 0 if x is 0. */
    inline double sign0(double x) {
        return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
    }

    inline double distance(double x0, double y0, double x1, double y1) {
        return hypot(x1 - x0, y1 - y0);
    }

    double Distance2Line(double pX, double pY, double x0, double y0, double x1, double y1);

    bool Intersect(std::vector<geometry_msgs::Point> &polygon, float testx, float testy);

    bool Intersect(std::vector<geometry_msgs::Point> &polygon1, std::vector<geometry_msgs::Point> &polygon2);
}
#endif //OR_COSTMAP_COSTMAP_MATH_H
