//
// Created by cxn on 2020/6/11.
//
/*
 * 水平太菜，制图的部分步骤需要手动完成
 * 程序启动后，会读取 存储了lm 坐标的txt文件，并自动生成地图，我们就设置地图默认大小为 740*400 分辨率为0.05
 *
 *  1. 左击，选定最近的lm，从0开始给lm标号； 按'a'键，标注完毕，清除其他lm；
 *      如要ICP计算，标号的时候需要从下往上标，在从左往右标，再按'b'键，用ICP计算模拟器中算法结果的误差
 *      例：
 *      A B C D
 *      E F G H
 *      应该按 E->A->F->B->G->C->H->D的顺序标注
 *  2. 右击按下到右击松开的两个点 最近的两个 lm 绘制连线
 *  3. 按'c'键，保存新地图txt、pgm文件
*/

#include "making_maps2.h"

using namespace std;
using namespace cv;

String g_window_name = "制图者";
vector<VisualizeLandMarks> g_lms;
vector<VisualizeLandMarks *> g_filter_lms;
VisualizeLandMarks *g_last_selected_lm;

int main() {
    //ICP函数测试
//    icp_function::IcpTest();

    //读取exe_config.yaml文件
    ExeConfig exe_config;
    ReadExeConfigFile(exe_config);

    Eigen::Vector2d origin;

    LoadMapFromTxt(exe_config.read_map_file_name, g_lms, origin, exe_config);

    MapSize mapSize;
    for (auto &item:g_lms) {
        mapSize.Expand(item.trueXY);
    }
    mapSize.Padding();
    Mat src_map(mapSize.height, mapSize.width, CV_8UC1, Scalar(255));

    for (auto &item:g_lms) {
        item.Drift(mapSize);
        circle(src_map, Point(item.x(), item.y()), exe_config.raduis, Scalar(0), -1);
    }

    namedWindow(g_window_name);
    setMouseCallback(g_window_name, OnMouseHandle, (void *) &src_map);

    cout << "*  1. 左击，选定最近的lm，从0开始给lm标号； 按'a'键，标注完毕，清除其他lm；" << endl;
    cout << "*      如要ICP计算，标号的时候需要从下往上标，在从左往右标，再按'b'键，用ICP计算模拟器中算法结果的误差" << endl;
    cout << "*      例：" << endl;
    cout << "*      A B C D" << endl;
    cout << "*      E F G H" << endl;
    cout << "*      应该按 E->A->F->B->G->C->H->D的顺序标注" << endl;
    cout << "*  2. 右击按下到右击松开的两个点 最近的两个 lm 绘制连线" << endl;
    cout << "*  3. 按'c'键，保存新地图txt、pgm文件" << endl;

    while (1) {
        imshow(g_window_name, src_map);
        auto key = waitKey(10);
        if (key == 97) {
            //a键 清除其余lm
            vector<VisualizeLandMarks> tmp;
            for (auto &item:g_filter_lms) {
                tmp.push_back(*item);
            }
            g_filter_lms.clear();
            g_lms = tmp;

            mapSize.Reset();
            for (auto &item:g_lms) {
                mapSize.Expand(item.driftXY);
            }
            mapSize.Padding();
            src_map = Mat(mapSize.height, mapSize.width, CV_8UC1, Scalar(255));
            for (auto &item:g_lms) {
                item.Drift(mapSize);
                circle(src_map, Point(item.x(), item.y()), exe_config.raduis, Scalar(0), -1);
            }

        } else if (key == 99) {
            //c键 save map
            SaveMaptoTxt(exe_config.out_map_file_name, g_lms, origin);
            std::cout << "Save map to " << exe_config.out_map_file_name << std::endl;
            imwrite(exe_config.out_map_pgm_name, src_map);
        } else if (key == 27) {
            //ESC 退出应用
            break;
        }
    }

    return 0;
}


void ReadExeConfigFile(ExeConfig &exe_config) {
    FileStorage fs("/home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_making_map/exe_config.yaml",
                   FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Open exe_config.yaml Failed!" << std::endl;
        return;
    }
    fs["solution"] >> exe_config.solution;
    fs["raduis"] >> exe_config.raduis;
    fs["read_map_file_name"] >> exe_config.read_map_file_name;
    fs["out_map_file_name"] >> exe_config.out_map_file_name;
    fs["out_map_pgm_name"] >> exe_config.out_map_pgm_name;
    fs.release();
}

void
LoadMapFromTxt(std::string filename, vector<VisualizeLandMarks> &lms, Eigen::Vector2d &origin, ExeConfig &exe_config) {
    std::ifstream f;
    f.open(filename.c_str());
    if (!f.is_open()) {
        std::cerr << " can't open map file " << std::endl;
        return;
    } else {
        std::cout << " Load map from: " << filename << std::endl;
    }

    bool begin = false;
    while (!f.eof()) {
        std::string s;
        std::getline(f, s);
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            if (!begin) {
                ss >> origin[0];
                ss >> origin[1];
                begin = true;
            } else {
                double x, y;
                int idx;
                ss >> x;
                ss >> y;
                ss >> idx;
                lms.emplace_back(x, y, idx);
            }
        }
    }
}

void SaveMaptoTxt(std::string filename, vector<VisualizeLandMarks> &lms, Eigen::Vector2d &origin) {
    if (lms.empty())
        return;

    std::ofstream foutC;
    foutC.open(filename.c_str());

    if (!foutC.is_open()) {
        std::cerr << " can't open cam pose file " << std::endl;
        return;
    }

    foutC.setf(std::ios::fixed, std::ios::floatfield);
    foutC.precision(5);

    origin = origin - lms[0].trueXY + lms[0].driftXY;

    foutC << origin[0] << " "
          << origin[1] << std::endl;

    for (size_t i = 0; i < lms.size(); ++i) {
        auto &T = lms.at(i);
        foutC << T.driftXY[0] << " "
              << T.driftXY[1] << " "
              << T.id << std::endl;
    }
    foutC.close();
}

//TODO,可以用kd-tree优化
int GetMinDistanceLm(int x, int y) {
    int min_dis = numeric_limits<int>::max();
    int min_index = -1;
    for (int i = 0; i < g_lms.size(); i++) {
        auto &lm = g_lms[i];
        int dis = (lm.x() - x) * (lm.x() - x) + (lm.y() - y) * (lm.y() - y);
        if (dis < min_dis) {
            min_dis = dis;
            min_index = i;
        }
    }
    return min_index;
}

void MarkMinDistanceLm(int x, int y) {
    int min_index = GetMinDistanceLm(x, y);
    if (min_index != -1) {
        g_filter_lms.push_back(&(g_lms[min_index]));
        g_filter_lms.back()->id = g_filter_lms.size() - 1;
    }
}


void OnMouseHandle(int event, int x, int y, int flag, void *param) {
    //先将param强制转化成一个mat类型的指针，然后引用指针指向的值（即image是*param的一个别名）
    Mat &image = *(Mat *) param;

    switch (event) {
        case EVENT_MOUSEMOVE: {
        }
            break;
        case EVENT_LBUTTONDOWN: {
            MarkMinDistanceLm(x, y);
            putText(image, to_string(g_filter_lms.size()), Point(x, y),
                    FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar::all(0), 2);
        }
            break;
        case EVENT_LBUTTONUP: {
        }
            break;
        case EVENT_RBUTTONDOWN: {
            int min_index = GetMinDistanceLm(x, y);
            if (min_index != -1)
                g_last_selected_lm = &(g_lms[min_index]);
        }
            break;
        case EVENT_RBUTTONUP: {
            int min_index = GetMinDistanceLm(x, y);
            if (min_index != -1 && g_last_selected_lm) {
                auto &this_selected = g_lms[min_index];
                line(image, Point(g_last_selected_lm->x(), g_last_selected_lm->y()),
                     Point(this_selected.x(), this_selected.y()), Scalar(0), 2);
                g_last_selected_lm = nullptr;
            }
        }
            break;
        default:
            break;
    }
}
