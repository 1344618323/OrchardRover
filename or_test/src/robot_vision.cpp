#include<ros/ros.h> //ros标准库头文件
#include<iostream> //C++标准输入输出库
/*
  cv_bridge中包含CvBridge库
*/
#include<cv_bridge/cv_bridge.h>
/*
  ROS图象类型的编码函数
*/
#include<sensor_msgs/image_encodings.h>
/*
   image_transport 头文件用来在ROS系统中的话题上发布和订阅图象消息
*/
#include<image_transport/image_transport.h>

//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

static const std::string INPUT = "Input"; //定义输入窗口名称
static const std::string OUTPUT = "Output"; //定义输出窗口名称

//定义一个转换的类
class RGB_GRAY {
private:
    ros::NodeHandle nh_; //定义ROS句柄
    image_transport::ImageTransport it_; //定义一个image_transport实例
    image_transport::Subscriber image_sub_; //定义ROS图象接收器
    //image_transport::Publisher image_pub_; //定义ROS图象发布器
public:
    RGB_GRAY() : it_(nh_) //构造函数
    {
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &RGB_GRAY::convert_callback,
                                   this); //定义图象接受器，订阅话题是“camera/rgb/image_raw”
        // image_pub_ = it_.publishe("", 1); //定义图象发布器
        //初始化输入输出窗口
        cv::namedWindow(INPUT);
        cv::namedWindow(OUTPUT);
    }

    ~RGB_GRAY() //析构函数
    {
        cv::destroyWindow(INPUT);
        cv::destroyWindow(OUTPUT);
    }

    /*
      这是一个ROS和OpenCV的格式转换回调函数，将图象格式从sensor_msgs/Image  --->  cv::Mat
    */
    void convert_callback(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例
        try {
            cv_ptr = cv_bridge::toCvCopy(msg,
                                         sensor_msgs::image_encodings::BGR8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
        }
        catch (cv_bridge::Exception &e)  //异常处理
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        image_process(cv_ptr->image); //得到了cv::Mat类型的图象，在CvImage指针的image中，将结果传送给处理函数   
    }

    /*
       这是图象处理的主要函数，一般会把图像处理的主要程序写在这个函数中。这里的例子只是一个彩色图象到灰度图象的转化
    */
    int i = 0;

    void image_process(cv::Mat img) {
        cv::Mat img_out;
        cv::cvtColor(img, img_out, CV_RGB2GRAY);  //转换成灰度图象
        cv::imshow(INPUT, img);
        // cv::imshow(OUTPUT, img_out);

        auto key = cv::waitKey(10);

        if (key == 13) {
            std::stringstream ss;
            ss << "/home/cxn/myfile/orchardrover_ws/src/OrchardRover/or_test/img/" << (i++) << ".jpg";
            imwrite(ss.str(), img);
        }
    }
};

//主函数
int main(int argc, char **argv) {
    ros::init(argc, argv, "RGB");
    RGB_GRAY obj;
    ros::spin();
}