//
// Created by cxn on 2020/6/24.
//

#ifndef MAKING_MAPS_MAKING_MAPS_H
#define MAKING_MAPS_MAKING_MAPS_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <memory>
#include "icpfun.h"

struct ExeConfig {
    double solution;
    int raduis;
    int img_width;
    int img_height;
    string read_map_file_name;
    string out_map_file_name;
    string out_map_pgm_name;
};

void WriteSimConfigFile();

void ReadExeConfigFile(ExeConfig &exe_config);

struct VisualizeLandMarks {
    VisualizeLandMarks(double x, double y, int idx, ExeConfig &sim_config) {
        trueXY.x() = x;
        trueXY.y() = y;
        id = idx;
        imgXY.x() = trueXY.x() / sim_config.solution;
        imgXY.y() = sim_config.img_height - trueXY.y() / sim_config.solution;
    }

    int x() {
        return imgXY.x();
    }

    int y() {
        return imgXY.y();
    }

    Eigen::Vector2d trueXY;
    Eigen::Vector2i imgXY;
    int id;
};

void LoadMapFromTxt(std::string filename, vector<VisualizeLandMarks> &lms, ExeConfig &exe_config);

void SaveMaptoTxt(std::string filename, vector<VisualizeLandMarks> &lms);


void OnMouseHandle(int event, int x, int y, int flag, void *param);

#endif //MAKING_MAPS_MAKING_MAPS_H
