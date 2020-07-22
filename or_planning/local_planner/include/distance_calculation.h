#ifndef OR_PLANNING_LOCAL_PLANNER_DISTANCE_CALCULATIONS_H
#define OR_PLANNING_LOCAL_PLANNER_DISTANCE_CALCULATIONS_H

#include <Eigen/Core>
#include "utility_tool.h"

namespace or_local_planner {

    /**
     * @brief This file is a distance calculator
     */

    //! A point container
    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Point2dContainer;

    /**
     * @brief Calculate a closest point from a point to 2D line segment
     * @param point Input point
     * @param line_start Start point of line
     * @param line_end End Point of line
     * @return Closest point on line
     */
    inline Eigen::Vector2d ClosestPointOnLineSegment2D(const Eigen::Ref<const Eigen::Vector2d> &point,
                                                       const Eigen::Ref<const Eigen::Vector2d> &line_start,
                                                       const Eigen::Ref<const Eigen::Vector2d> &line_end) {
        Eigen::Vector2d diff = line_end - line_start;
        double sq_norm = diff.squaredNorm();

        if (sq_norm == 0) {
            return line_start;
        }

        double u = ((point.x() - line_start.x()) * diff.x() + (point.y() - line_start.y()) * diff.y()) / sq_norm;

        if (u <= 0) {
            return line_start;
        } else if (u >= 1) {
            return line_end;
        }

        return line_start + u * diff;
    }

    /**
     * @brief Calculate distance from point to 2D line segment
     * @param point Input point
     * @param line_start Start point of line
     * @param line_end End Point of line
     * @return Distance
     */

    inline double DistancePointToSegment2D(const Eigen::Ref<const Eigen::Vector2d> &point,
                                           const Eigen::Ref<const Eigen::Vector2d> &line_start,
                                           const Eigen::Ref<const Eigen::Vector2d> &line_end) {
        return (point - ClosestPointOnLineSegment2D(point, line_start, line_end)).norm();
    }

    /**
     * @brief Calculate if two 2D line segment intersection
     * @param line1_start First line start point
     * @param line1_end Firts line end point
     * @param line2_start Second line start point
     * @param line2_end Second line end point
     * @param intersection Point where intersection
     * @return If true intersection, else not intersection
     */
    inline bool CheckLineSegmentsIntersection2D(const Eigen::Ref<const Eigen::Vector2d> &line1_start,
                                                const Eigen::Ref<const Eigen::Vector2d> &line1_end,
                                                const Eigen::Ref<const Eigen::Vector2d> &line2_start,
                                                const Eigen::Ref<const Eigen::Vector2d> &line2_end,
                                                Eigen::Vector2d *intersection = NULL) {
        double s_numer, t_numer, denom, t;
        Eigen::Vector2d line1 = line1_end - line1_start;
        Eigen::Vector2d line2 = line2_end - line2_start;

        denom = line1.x() * line2.y() - line2.x() * line1.y();
        if (denom == 0) {
            return false;
        }
        bool denomPositive = denom > 0;

        Eigen::Vector2d aux = line1_start - line2_start;

        s_numer = line1.x() * aux.y() - line1.y() * aux.x();
        if ((s_numer < 0) == denomPositive) {
            return false;
        }

        t_numer = line2.x() * aux.y() - line2.y() * aux.x();
        if ((t_numer < 0) == denomPositive) {
            return false;
        }

        if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive)) {
            return false;
        }

        t = t_numer / denom;
        if (intersection) {
            *intersection = line1_start + t * line1;
        }

        return true;
    }

    /**
     * @brief Calculate two 2D segment lines' distance
     * @param line1_start First line start point
     * @param line1_end Firts line end point
     * @param line2_start Second line start point
     * @param line2_end Second line end point
     * @return distance of two 2D segment lines
     */

    inline double DistanceSegmentToSegment2D(const Eigen::Ref<const Eigen::Vector2d> &line1_start,
                                             const Eigen::Ref<const Eigen::Vector2d> &line1_end,
                                             const Eigen::Ref<const Eigen::Vector2d> &line2_start,
                                             const Eigen::Ref<const Eigen::Vector2d> &line2_end) {

        if (CheckLineSegmentsIntersection2D(line1_start, line1_end, line2_start, line2_end)) {
            return 0;
        }

        std::array<double, 4> distances;

        distances[0] = DistancePointToSegment2D(line1_start, line2_start, line2_end);
        distances[1] = DistancePointToSegment2D(line1_end, line2_start, line2_end);
        distances[2] = DistancePointToSegment2D(line2_start, line1_start, line1_end);
        distances[3] = DistancePointToSegment2D(line2_end, line1_start, line1_end);

        return *std::min_element(distances.begin(), distances.end());
    }

    /**
     * @brief Calculate point to 2D polygon distance
     * @param point Input point
     * @param vertices 2D polygon's vertices
     * @return distance of point to 2D polygon
     */
    inline double DistancePointToPolygon2D(const Eigen::Vector2d &point, const Point2dContainer &vertices) {
        double dist = HUGE_VAL;

        if (vertices.size() == 1) {
            return (point - vertices.front()).norm();
        }

        for (int i = 0; i < (int) vertices.size() - 1; ++i) {

            double new_dist = PointToLineDistance(point.coeffRef(0), point.coeffRef(1), vertices.at(i).coeffRef(0),
                                                  vertices.at(i).coeffRef(1),
                                                  vertices.at(i + 1).coeffRef(0), vertices.at(i + 1).coeffRef(1));
            if (new_dist < dist) {
                dist = new_dist;
            }

        }

        /*if (vertices.size() > 2) {
          double new_dist = PointToLineDistance(point.coeffRef(0), point.coeffRef(1), vertices.back().coeffRef(0), vertices.back().coeffRef(1),
                                                vertices.front().coeffRef(0), vertices.front().coeffRef(1));
          if (new_dist < dist) {
            std::cout << "here" << std::endl;
            dist = new_dist;
          }
        }*/

        return dist;
    }

    /**
     * @brief Calculate 2D line segment to 2D polygon distance
     * @param line_start Line start point
     * @param line_end Line end point
     * @param vertices Vertices 2D polygon's vertices
     * @return distance of 2D line segment to 2D polygon
     */
    inline double DistanceSegmentToPolygon2D(const Eigen::Vector2d &line_start,
                                             const Eigen::Vector2d &line_end,
                                             const Point2dContainer &vertices) {
        double dist = HUGE_VAL;

        if (vertices.size() == 1) {
            return DistancePointToSegment2D(vertices.front(), line_start, line_end);
        }

        for (int i = 0; i < (int) vertices.size() - 1; ++i) {
            double new_dist = DistanceSegmentToSegment2D(line_start, line_end, vertices.at(i), vertices.at(i + 1));
            if (new_dist < dist) {
                dist = new_dist;
            }
        }

        if (vertices.size() > 2) {
            double new_dist =
                    DistanceSegmentToSegment2D(line_start, line_end, vertices.back(), vertices.front());
            if (new_dist < dist)
                return new_dist;
        }

        return dist;
    }

    /**
     * @brief Calculate two 2D polygons' distance
     * @param vertices1 Vertices of first 2D polygon's vertices
     * @param vertices2 Vertices of second 2D polygon's vertices
     * @return distance of two 2D polygons
     */
    inline double DistancePolygonToPolygon2D(const Point2dContainer &vertices1, const Point2dContainer &vertices2) {
        double dist = HUGE_VAL;

        if (vertices1.size() == 1) {
            return DistancePointToPolygon2D(vertices1.front(), vertices2);
        }

        for (int i = 0; i < (int) vertices1.size() - 1; ++i) {
            double new_dist = DistanceSegmentToPolygon2D(vertices1[i], vertices1[i + 1], vertices2);
            if (new_dist < dist) {
                dist = new_dist;
            }
        }

        if (vertices1.size() > 2) {
            double new_dist = DistanceSegmentToPolygon2D(vertices1.back(), vertices1.front(), vertices2);
            if (new_dist < dist) {
                return new_dist;
            }
        }

        return dist;
    }

} // namespace or_local_planner
#endif // or_PLANNING_LOCAL_PLANNER_DISTANCE_CALCULATIONS_H
