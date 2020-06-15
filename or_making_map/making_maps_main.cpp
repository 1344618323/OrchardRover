//
// Created by cxn on 2020/6/11.
//
/*
 * 水平太菜，制图的部分步骤需要手动完成
 * 程序启动后，会读取 存储了lm 坐标的 csv文件，并自动生成地图，我们就设置地图默认大小为 740*400 分辨率为0.05
 * 'a'键（默认）状态下
 *      左击：清除最近的 lm
 *      右击按下到右击松开的两个点 最近的两个 lm 绘制连线
 * 'b'键 左击：给lm标号，从0开始标注；标注完毕后，按'c'键，用ICP计算模拟器中算法结果的误差
*/

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <memory>
#include "writetxt.h"

namespace icp_function {
    Eigen::Vector2d IcpMean(vector<Eigen::Vector2d> &src, vector<Eigen::Vector2d> &q_src) {
        q_src = src;
        Eigen::Vector2d val;
        for (auto &i:src) {
            val += i;
        }
        val = val / src.size();
        for (auto &i:q_src) {
            i -= val;
        }
        return val;
    }

    double ICP2D(std::vector<Eigen::Vector2d> &src, vector<Eigen::Vector2d> &dst) {
        vector<Eigen::Vector2d> q_src;
        vector<Eigen::Vector2d> q_dst;
        Eigen::Vector2d q_mean_src = icp_function::IcpMean(src, q_src);
        Eigen::Vector2d q_mean_dst = icp_function::IcpMean(dst, q_dst);

        Eigen::Matrix2d W;
        W.setZero();
        for (int i = 0; i < q_src.size(); i++) {
            W += q_src[i] * q_dst[i].transpose();
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix2d V = svd.matrixV(), U = svd.matrixU();
        Eigen::Matrix2d R = U * (V.transpose());
        Eigen::Vector2d t = q_mean_src - R * q_mean_dst;

        double e = 0;
        for (int i = 0; i < q_src.size(); i++) {
            e += (src[i] - R * dst[i] - t).norm();
        }
        double error = e / double(src.size());
        cout << error << endl;
        return error;
    }

    void IcpTest() {
        vector<Eigen::Vector2d> src;
        vector<Eigen::Vector2d> dst;
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 4; j++) {
                src.emplace_back(i * 3 + 6.5, j * 3 + 5);
            }
        }
        dst.emplace_back(6.51557, 4.72049);
        dst.emplace_back(6.48971, 7.71136);
        dst.emplace_back(6.54979, 10.7426);
        dst.emplace_back(6.54465, 13.7616);
        dst.emplace_back(9.42711, 4.58894);
        dst.emplace_back(9.5004, 7.6019);
        dst.emplace_back(9.49539, 10.6225);
        dst.emplace_back(9.56029, 13.6675);
        dst.emplace_back(12.3852, 4.6121);
        dst.emplace_back(12.4341, 7.63565);
        dst.emplace_back(12.3126, 10.6401);
        dst.emplace_back(12.5647, 13.6694);
        dst.emplace_back(15.3754, 4.55501);
        dst.emplace_back(15.4511, 7.56979);
        dst.emplace_back(15.5436, 10.6915);
        dst.emplace_back(15.6002, 13.7607);
        dst.emplace_back(18.3913, 4.63267);
        dst.emplace_back(18.4197, 7.64599);
        dst.emplace_back(18.5938, 10.938);
        dst.emplace_back(18.5292, 13.9812);
        dst.emplace_back(21.3952, 4.63682);
        dst.emplace_back(21.3932, 7.66796);
        dst.emplace_back(21.5391, 10.7096);
        dst.emplace_back(21.7621, 13.7351);
        dst.emplace_back(24.3903, 4.60888);
        dst.emplace_back(24.4003, 7.6429);
        dst.emplace_back(24.5718, 10.6763);
        dst.emplace_back(24.6453, 13.7544);

        ICP2D(src, dst);
    }
};

int raduis = 4;//树木的半径，×0.05
int img_width = 740;
int img_height = 400;
double solution = 0.05;

struct LandMark {
    LandMark(Eigen::Vector2d &xy) {
        trueXY = xy;
        imgXY.x() = trueXY.x() / solution;
        imgXY.y() = img_height - trueXY.y() / solution;
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

String window_name = "制图者";

void OnMouseHandle(int event, int x, int y, int flag, void *param);

int main() {
    ifstream inFile("/home/cxn/error.csv", ios::in);
    string lineStr;
    vector<LandMark> src_lms;

    int line = 0;

    while (getline(inFile, lineStr)) {
        //第一行跳过
        if (line == 0) {
            line++;
            continue;
        }
        stringstream ss(lineStr);
        string str;
        Eigen::Vector2d lm;
        // 按照逗号分隔
        getline(ss, str, ',');
        lm.x() = atof(str.c_str());
        getline(ss, str, ',');
        lm.y() = atof(str.c_str());
        src_lms.emplace_back(lm);
    }

    Mat src_map(img_height, img_width, CV_8UC1, Scalar(255));

    for (auto &lm:src_lms) {
        circle(src_map, Point(lm.x(), lm.y()), raduis, Scalar(0), -1);
        cout << lm.x() << " " << lm.y() << endl;
    }

    namedWindow(window_name);
    setMouseCallback(window_name, OnMouseHandle, (void *) &src_map);

    while (1) {
//        if (m_drawBox)
//            DrawRect(tempImg, m_rect);
        imshow(window_name, src_map);

        auto key = waitKey(10);
        if (key == 97) {
            break;
        } else if (key == 98) {

        } else if (key == 27) {
            break;
        }
    }

    return 0;
}


void OnMouseHandle(int event, int x, int y, int flag, void *param) {
    //先将param强制转化成一个mat类型的指针，然后引用指针指向的值（即image是*param的一个别名）
    Mat &image = *(Mat *) param;
    //Mat *image = (Mat*) param;//与上述语句功能等价

    switch (event) {
        case EVENT_MOUSEMOVE: {
//            if (m_drawBox) {
//                m_rect.width = x - m_rect.x;
//                m_rect.height = x - m_rect.y;
//            }
        }
            break;
        case EVENT_LBUTTONDOWN: {
//            m_drawBox = true;
//            m_rect = Rect(x, y, 0, 0);
        }
            break;
        case EVENT_LBUTTONUP: {
//            m_drawBox = false;
//            if (m_rect.width < 0) {
//                m_rect.x += m_rect.width;
//                m_rect.width *= -1;
//            }
//            if (m_rect.height < 0) {
//                m_rect.y += m_rect.height;
//                m_rect.height *= -1;
//            }
//
//            DrawRect(image, m_rect);
//            m_rect = Rect(-1, -1, 0, 0);
        }
        default:
            break;
    }
}



//Rect m_rect;
//bool m_drawBox = false;
//RNG m_rng(12345);
//
//String windowName = "鼠标拖动绘制矩形";
////-----------------------【全局函数声明部分】-------------------------
//void on_mouseHandle(int event, int x, int y, int flag, void *param);
//
//void DrawRect(Mat &img, Rect box);
//
////-----------------------【main函数部分】-----------------------------
//int main(int argc, char *argv[]) {
//    m_rect = Rect(-1, -1, 0, 0);
//    Mat srcImg(600, 800, CV_8UC3), tempImg;
//    srcImg.copyTo(tempImg);
//    srcImg = Scalar::all(0);
//
//    namedWindow(windowName);
//    /*--------------------【setMouseCallback说明】--------------------
//      函数void setMouseCallback(conststring& winname,//窗口的名字
//                                MouseCallback onMouse,//指定窗口里每次鼠标时间发生的时候，被调用的函数指针
//                                void* userdata=0)；//用户定义的传递到回调函数的参数，有默认值0
//      onMouse函数的原型应该为voidFoo(int event, int x, int y, int flags, void* param);
//      其中event是 CV_EVENT_*变量之一，
//      x和y是鼠标指针在图像坐标系的坐标（不是窗口坐标系），
//      flags是CV_EVENT_FLAG的组合，
//      param是用户定义的传递到cvSetMouseCallback函数调用的参数。
//    -------------------------------------------------------------------*/
//    setMouseCallback(windowName, on_mouseHandle, (void *) &srcImg);
//
//    while (1) {
//        srcImg.copyTo(tempImg);
//        if (m_drawBox) DrawRect(tempImg, m_rect);
//        imshow(windowName, srcImg);
//        if (waitKey(10) == 27) {
//            imwrite("1.png", srcImg);
//            break;
//        }
//    }
//    return 0;
//}
//
////-----------------------【on_mouseHandle函数】---------------------------

//
//
////-----------------------【DrawRect函数】---------------------------
//void DrawRect(Mat &img, Rect box) {
//    rectangle(img, box.tl(), box.br(),
//              Scalar(m_rng.uniform(0, 255), m_rng.uniform(0, 255), m_rng.uniform(0, 255)));
//}
