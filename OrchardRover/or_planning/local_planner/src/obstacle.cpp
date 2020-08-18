//
// Created by cxn on 2020/7/8.
//

#include "obstacle.h"

namespace or_local_planner {

    void PolygonObstacle::FixPolygonClosure() {
        if (vertices_.size() < 2)
            return;

        if (vertices_.front().isApprox(vertices_.back()))
            vertices_.pop_back();
    }

    void PolygonObstacle::CalcCentroid() {
        if (vertices_.empty()) {
            centroid_.setConstant(NAN);
            ROS_WARN("can't compute the centriod because the vertices is null");
            return;
        }

        if (NoVertices() == 1) {
            centroid_ = vertices_.front();
            return;
        }

        if (NoVertices() == 2) {
            centroid_ = 0.5 * (vertices_.front() + vertices_.back());
            return;
        }


        centroid_.setZero();

        // calculate centroid (see wikipedia http://de.wikipedia.org/wiki/Geometrischer_Schwerpunkt#Polygon)
        double A = 0;  // A = 0.5 * sum_0_n-1 (x_i * y_{i+1} - x_{i+1} * y_i)
        for (int i = 0; i < NoVertices() - 1; ++i) {
            A += vertices_.at(i).coeffRef(0) * vertices_.at(i + 1).coeffRef(1)
                 - vertices_.at(i + 1).coeffRef(0) * vertices_.at(i).coeffRef(1);
        }
        A += vertices_.at(NoVertices() - 1).coeffRef(0) * vertices_.at(0).coeffRef(1)
             - vertices_.at(0).coeffRef(0) * vertices_.at(NoVertices() - 1).coeffRef(1);
        A *= 0.5;

        if (A != 0) {
            for (int i = 0; i < NoVertices() - 1; ++i) {
                double aux = (vertices_.at(i).coeffRef(0) * vertices_.at(i + 1).coeffRef(1)
                              - vertices_.at(i + 1).coeffRef(0) * vertices_.at(i).coeffRef(1));
                centroid_ += (vertices_.at(i) + vertices_.at(i + 1)) * aux;
            }
            double aux = (vertices_.at(NoVertices() - 1).coeffRef(0) * vertices_.at(0).coeffRef(1)
                          - vertices_.at(0).coeffRef(0) * vertices_.at(NoVertices() - 1).coeffRef(1));
            centroid_ += (vertices_.at(NoVertices() - 1) + vertices_.at(0)) * aux;
            centroid_ /= (6 * A);
        } else {
            int i_cand = 0;
            int j_cand = 0;
            double min_dist = std::numeric_limits<double>::max();
            for (int i = 0; i < NoVertices(); ++i) {
                for (int j = i + 1; j < NoVertices(); ++j) {
                    double dist = (vertices_[j] - vertices_[i]).norm();
                    if (dist < min_dist) {
                        min_dist = dist;
                        i_cand = i;
                        j_cand = j;
                    }
                }
            }
            centroid_ = 0.5 * (vertices_[i_cand] + vertices_[j_cand]);
        }
    }

    Eigen::Vector2d PolygonObstacle::GetClosestPoint(const Eigen::Vector2d &position) const {
        if (NoVertices() == 1) {
            return vertices_.front();
        }

        if (NoVertices() > 1) {

            Eigen::Vector2d new_pt = ClosestPointOnLineSegment2D(position, vertices_.at(0), vertices_.at(1));

            if (NoVertices() > 2) {
                double dist = (new_pt - position).norm();
                Eigen::Vector2d closest_pt = new_pt;

                for (int i = 1; i < NoVertices() - 1; ++i) {
                    new_pt = ClosestPointOnLineSegment2D(position, vertices_.at(i), vertices_.at(i + 1));
                    double new_dist = (new_pt - position).norm();
                    if (new_dist < dist) {
                        dist = new_dist;
                        closest_pt = new_pt;
                    }
                }
                new_pt = ClosestPointOnLineSegment2D(position, vertices_.back(), vertices_.front());
                double new_dist = (new_pt - position).norm();
                if (new_dist < dist)
                    return new_pt;
                else
                    return closest_pt;
            } else {
                return new_pt;
            }
        }

        return Eigen::Vector2d::Zero();
    }

    bool PolygonObstacle::CheckLineIntersection(const Eigen::Vector2d &line_start,
                                                const Eigen::Vector2d &line_end,
                                                double min_dist) const {

        for (int i = 0; i < NoVertices() - 1; ++i) {
            if (CheckLineSegmentsIntersection2D(line_start, line_end, vertices_.at(i), vertices_.at(i + 1))) {
                return true;
            }
        }
        if (NoVertices() == 2) {
            return false;
        }

        return CheckLineSegmentsIntersection2D(line_start,
                                               line_end,
                                               vertices_.back(),
                                               vertices_.front());
    }

    void PolygonObstacle::ToPolygonMsg(geometry_msgs::Polygon &polygon) {
        polygon.points.resize(vertices_.size());
        for (std::size_t i = 0; i < vertices_.size(); ++i) {
            polygon.points[i].x = vertices_[i].x();
            polygon.points[i].y = vertices_[i].y();
            polygon.points[i].z = 0;
        }
    }
}