//
// Created by cxn on 2020/7/1.
//

#ifndef OR_COSTMAP_FOOTPRINT_H
#define OR_COSTMAP_FOOTPRINT_H

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include "costmap_math.h"

namespace or_costmap {
    /**
     * @brief Calculate the extreme distances for the footprint
     *
     * @param footprint The footprint to examine
     * @param min_dist Output parameter of the minimum distance
     * @param max_dist Output parameter of the maximum distance
     * 计算footprint的内接圆、外接圆
     */
    void CalculateMinAndMaxDistances(const std::vector<geometry_msgs::Point> &footprint,
                                     double &min_dist, double &max_dist);

    /**
     * @brief Convert Point32 to Point
     */
    geometry_msgs::Point ToPoint(geometry_msgs::Point32 pt);

    /**
     * @brief Convert Point to Point32
     */
    geometry_msgs::Point32 ToPoint32(geometry_msgs::Point pt);

    /**
     * @brief Convert vector of Points to Polygon msg
     */
    geometry_msgs::Polygon ToPolygon(std::vector<geometry_msgs::Point> pts);

    /**
     * @brief Convert Polygon msg to vector of Points.
     */
    std::vector<geometry_msgs::Point> ToPointVector(geometry_msgs::Polygon polygon);

    /**
     * @brief  Given a pose and base footprint, build the oriented footprint of the robot (list of Points)
     * @param  x The x position of the robot
     * @param  y The y position of the robot
     * @param  theta The orientation of the robot
     * @param  footprint_spec Basic shape of the footprint
     * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
     * 输入机器人坐标(x,y,theta)，求机器人footprint坐标
    */
    void TransformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point> &footprint_spec,
                            std::vector<geometry_msgs::Point> &oriented_footprint);

    /**
     * @brief  Given a pose and base footprint, build the oriented footprint of the robot (PolygonStamped)
     * @param  x The x position of the robot
     * @param  y The y position of the robot
     * @param  theta The orientation of the robot
     * @param  footprint_spec Basic shape of the footprint
     * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
     * 输入机器人坐标(x,y,theta)，求机器人footprint坐标
    */
    void TransformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point> &footprint_spec,
                            geometry_msgs::PolygonStamped &oriented_footprint);

    /**
     * @brief Adds the specified amount of padding to the footprint (in place)
     * 输出扩增后的footprint
     */
    void PadFootprint(std::vector<geometry_msgs::Point> &footprint, double padding);
}

#endif //OR_COSTMAP_FOOTPRINT_H
