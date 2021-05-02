//
// Created by cxn on 2021/5/2.
//

#include "making_map_from_gmapping.h"

using namespace cv;
using namespace std;

struct RoiSize {
    int lx = numeric_limits<int>::max();
    int rx = numeric_limits<int>::min();
    int ty = numeric_limits<int>::max();
    int by = numeric_limits<int>::min();

    void draft(int col, int row) {
        if (col < lx)
            lx = col;
        if (col > rx)
            rx = col;
        if (row < ty)
            ty = row;
        if (row > by)
            by = row;
    }
};

RoiSize roi_size;
vector<Eigen::Vector2d> g_lms;

Eigen::Vector2d GetNeighorTrunk(Mat &img, int col, int row, RoiSize &roi_size, int r = 20) {
    Eigen::Vector2d mean;
    mean.setZero();
    int cnt = 0;
    for (int x = col - r; x < col + r; x++) {
        for (int y = row - r; y < row + r; y++) {
            if (img.at<uchar>(y, x) < 100) {
                mean.x() += x;
                mean.y() += y;
                cnt++;
                roi_size.draft(x, y);
            }
        }
    }
    if (cnt != 0) {
        mean.x() /= cnt;
        mean.y() /= cnt;
    }
    return mean;
}

void OnMouseHandle(int event, int x, int y, int flag, void *param) {
    //先将param强制转化成一个mat类型的指针，然后引用指针指向的值（即image是*param的一个别名）
    Mat &image = *(Mat *) param;

    switch (event) {
        case EVENT_MOUSEMOVE: {

        }
            break;
        case EVENT_LBUTTONDOWN: {
            Eigen::Vector2d out = GetNeighorTrunk(image, x, y, roi_size);
            if (out.x() != 0 || out.y() != 0) {
                g_lms.push_back(out);
                cout << out.x() << " " << out.y() << endl;
                cout << roi_size.lx << " " << roi_size.rx << " " << roi_size.ty << " " << roi_size.by << endl;
            }
        }
            break;
        case EVENT_LBUTTONUP: {
        }
            break;
        case EVENT_RBUTTONDOWN: {

        }
            break;
        case EVENT_RBUTTONUP: {

        }
            break;
        default:
            break;
    }
}

void SaveMaptoTxt(std::string filename, vector<Eigen::Vector2d> &lms, Eigen::Vector2d &origin) {
    std::ofstream foutC;
    foutC.open(filename.c_str());

    if (!foutC.is_open()) {
        std::cerr << " can't open cam pose file " << std::endl;
        return;
    }

    foutC.setf(std::ios::fixed, std::ios::floatfield);
    foutC.precision(5);

    foutC << 0 << " "
          << 0 << std::endl;

    for (size_t i = 0; i < lms.size(); ++i) {
        auto &T = lms.at(i) - origin;
        foutC << T.x() * 0.05 << " "
              << -T.y() * 0.05 << " "
              << i << std::endl;
    }
    foutC.close();
}


int main() {
    Mat src_map = imread("/home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/gmapping_map.pgm",
                         0);
//    cout <<"类型是" <<gmapping_map.type() << " " <<"通道数是"<<gmapping_map.channels() << endl;
    String g_window_name = "MakingMap";
    Mat gmapping_map = src_map;
    namedWindow(g_window_name, WINDOW_KEEPRATIO);
    setMouseCallback(g_window_name, OnMouseHandle, (void *) &gmapping_map);

    while (1) {
        imshow(g_window_name, gmapping_map);
        auto key = waitKey(10);
        if (key == 97) {
            //a键，重新制图
            for (int col = roi_size.lx; col <= roi_size.rx; col++) {
                for (int row = roi_size.ty; row <= roi_size.by; row++) {
                    gmapping_map.at<uchar>(row, col) = 255;
                }
            }
            for (auto &item:g_lms) {
                circle(gmapping_map, Point(round(item.x()), round(item.y())), 2, Scalar(0), -1);
            }

        } else if (key == 99) {
            //c键 save map
            if (!g_lms.empty()) {
                imwrite("/home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/my_map2.pgm",
                        gmapping_map);

                string filename = "/home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/true_map.txt";
                Eigen::Vector2d origin;
                origin.x() = gmapping_map.cols / 2;
                origin.y() = gmapping_map.rows / 2;
                SaveMaptoTxt(filename, g_lms, origin);
            }
        } else if (key == 27) {
            //ESC 退出应用
            break;
        }
    }

    return 0;
}
