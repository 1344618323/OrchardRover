#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "ros/ros.h"
#include <stdio.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#define MAX_IMAGE_DATA_SIZE (20 * 1024 * 1024)
#define IMAGE_DATA_SIZE (10 * 1024 * 1024)
#define IMAGE_SAVE_SIZE (10 * 1024 * 1024)

enum CamerProperties
{
    CAP_PROP_FRAMERATE_ENABLE,  //帧数可调
    CAP_PROP_FRAMERATE,         //帧数
    CAP_PROP_HEIGHT,            //图像高度
    CAP_PROP_WIDTH,             //图像宽度
    CAP_PROP_EXPOSURE_TIME,     //曝光时间
    CAP_PROP_GAMMA_ENABLE,      //伽马因子可调
    CAP_PROP_GAMMA,             //伽马因子
    CAP_PROP_GAIN,              //亮度
    CAP_PROP_SATURATION_ENABLE, //饱和度可调
    CAP_PROP_SATURATION         //饱和度

};

class Camera
{
public:
    //构造函数
    Camera();
    //析构函数
    ~Camera();
    //原始信息转换线程
    void *HKWorkThread(void *p_handle);
    //输出摄像头信息
    bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
    //设置摄像头参数
    bool set(CamerProperties type, float value);
    //恢复默认参数
    bool reset();

private:
    ros::NodeHandle hk_cam_node;
    cv::Mat frame;
    std::shared_ptr<image_transport::ImageTransport> cam_img_trans;
    image_transport::Publisher image_pub;
    

    void *handle;
    pthread_t nThreadID;
    int nRet;

    int width;
    int height;
    int FrameRateEnable;
    int FrameRate;
    int ExposureTime;
    int GammaEnable;
    float Gamma;
    float Gain;
    int SaturationEnable;
    int Saturation;
};
#endif