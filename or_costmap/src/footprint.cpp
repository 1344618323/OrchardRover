//
// Created by cxn on 2020/7/1.
//

#include "footprint.h"

namespace or_costmap {
    //求外切圆内切圆
    void CalculateMinAndMaxDistances(const std::vector<geometry_msgs::Point> &footprint,
                                     double &min_dist,
                                     double &max_dist) {
        min_dist = std::numeric_limits<double>::max();
        max_dist = 0.0;

        if (footprint.size() <= 2) {
            return;
        }

        for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
            // check the distance from the robot center point to the first vertex
            double vertex_dist = distance(0.0, 0.0, footprint[i].x, footprint[i].y);
            double edge_dist = Distance2Line(0.0, 0.0, footprint[i].x, footprint[i].y,
                                             footprint[i + 1].x, footprint[i + 1].y);
            min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
            max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
        }

        // we also need to do the last vertex and the first vertex
        double vertex_dist = distance(0.0, 0.0, footprint.back().x, footprint.back().y);
        double edge_dist = Distance2Line(0.0, 0.0, footprint.back().x, footprint.back().y,
                                         footprint.front().x, footprint.front().y);
        min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
        max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
    }


    geometry_msgs::Point32 ToPoint32(geometry_msgs::Point pt) {
        geometry_msgs::Point32 point32;
        point32.x = pt.x;
        point32.y = pt.y;
        point32.z = pt.z;
        return point32;
    }

    geometry_msgs::Point ToPoint(geometry_msgs::Point32 pt) {
        geometry_msgs::Point point;
        point.x = pt.x;
        point.y = pt.y;
        point.z = pt.z;
        return point;
    }

    geometry_msgs::Polygon ToPolygon(std::vector<geometry_msgs::Point> pts) {
        geometry_msgs::Polygon polygon;
        for (int i = 0; i < pts.size(); i++) {
            polygon.points.push_back(ToPoint32(pts[i]));
        }
        return polygon;
    }

    std::vector<geometry_msgs::Point> ToPointVector(geometry_msgs::Polygon polygon) {
        std::vector<geometry_msgs::Point> pts;
        for (int i = 0; i < polygon.points.size(); i++) {
            pts.push_back(ToPoint(polygon.points[i]));
        }
        return pts;
    }

    //计算[cos(theta) -s(theta); s(theta) c(theta)]*footprint_spec+[x,y] = oriented_footprint
    void TransformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point> &footprint_spec,
                            std::vector<geometry_msgs::Point> &oriented_footprint) {
        // build the oriented footprint at a given location
        oriented_footprint.clear();
        double cos_th = cos(theta);
        double sin_th = sin(theta);
        for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
            geometry_msgs::Point new_pt;
            new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
            new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
            oriented_footprint.push_back(new_pt);
        }
    }

    void TransformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point> &footprint_spec,
                            geometry_msgs::PolygonStamped &oriented_footprint) {
        // build the oriented footprint at a given location
        oriented_footprint.polygon.points.clear();
        double cos_th = cos(theta);
        double sin_th = sin(theta);
        for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
            geometry_msgs::Point32 new_pt;
            new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
            new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
            oriented_footprint.polygon.points.push_back(new_pt);
        }
    }

    void PadFootprint(std::vector<geometry_msgs::Point> &footprint, double padding) {
        // pad footprint in place
        for (unsigned int i = 0; i < footprint.size(); i++) {
            geometry_msgs::Point &pt = footprint[i];
            pt.x += sign0(pt.x) * padding;
            pt.y += sign0(pt.y) * padding;
        }
    }
}