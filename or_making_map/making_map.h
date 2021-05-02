//
// Created by cxn on 2020/6/24.
//

#ifndef MAKING_MAP_H
#define MAKING_MAP_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <memory>
#include <string>

using namespace std;

struct ExeConfig {
    double solution;
    double radius;
    string slam_map_file_name;
    string correct_map_file_name;
    string correct_map_pgm_name;
    string true_map_file_name;
};

struct MapSize {
    MapSize(double solution = 0.05) {
        this->solution = solution;
        min_val_x = numeric_limits<double>::max();
        max_val_x = numeric_limits<double>::min();
        min_val_y = numeric_limits<double>::max();
        max_val_y = numeric_limits<double>::min();
    }

    void Reset() {
        min_val_x = numeric_limits<double>::max();
        max_val_x = numeric_limits<double>::min();
        min_val_y = numeric_limits<double>::max();
        max_val_y = numeric_limits<double>::min();
    }

    void Expand(Eigen::Vector2d xy) {
        double x = xy.x();
        double y = xy.y();
        if (min_val_x > x)
            min_val_x = x;
        if (max_val_x < x)
            max_val_x = x;
        if (min_val_y > y)
            min_val_y = y;
        if (max_val_y < y)
            max_val_y = y;
    }

    void Padding(double pad = 3) {
        min_val_x -= pad;
        min_val_y -= pad;
        max_val_x += pad;
        max_val_y += pad;
        width = (max_val_x - min_val_x) / solution;
        height = (max_val_y - min_val_y) / solution;
    }

    double min_val_x;
    double min_val_y;
    double max_val_x;
    double max_val_y;
    int width;
    int height;
    double solution;
};

struct VisualizeLandMarks {
    VisualizeLandMarks(double x, double y, int idx) {
        trueXY.x() = x;
        trueXY.y() = y;
        id = idx;
        driftXY = trueXY;
    }

    void Drift(MapSize &mapSize) {
        driftXY.x() -= mapSize.min_val_x;
        driftXY.y() -= mapSize.min_val_y;
        imgXY.x() = driftXY.x() / mapSize.solution;
        imgXY.y() = mapSize.height - driftXY.y() / mapSize.solution;
    }

    int x() {
        return imgXY.x();
    }

    int y() {
        return imgXY.y();
    }


    Eigen::Vector2d trueXY;
    Eigen::Vector2d driftXY;
    Eigen::Vector2i imgXY;
    int id;
};

void ReadExeConfigFile(ExeConfig &exe_config);

void
LoadMapFromTxt(std::string filename, vector<VisualizeLandMarks> &lms, Eigen::Vector2d &origin, ExeConfig &exe_config);

void SaveMaptoTxt(std::string filename, vector<VisualizeLandMarks> &lms, Eigen::Vector2d &origin);

void OnMouseHandle(int event, int x, int y, int flag, void *param);

#endif //MAKING_MAPS_MAKING_MAP_H
