#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "ros/ros.h"
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"

namespace hk_camera {
#define MAX_IMAGE_DATA_SIZE (4 * 1440 * 1080)
#define IMAGE_DATA_SIZE (4 * 1440 * 1080)
#define IMAGE_SAVE_SIZE (4 * 1440 * 1080)

    // cv::Mat frame;
    // bool frame_empty = 0;

    enum CamerProperties {
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

    class HkCamera {
    public:
        //构造函数
        HkCamera(ros::NodeHandle &nh);

        //析构函数
        ~HkCamera();

        //输出摄像头信息
        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);

        //设置摄像头参数
        bool set(hk_camera::CamerProperties type, float value);

        //恢复默认参数
        bool reset();

        //读图
        void operator>>(cv::Mat &image);

    private:
        void *handle;
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

        unsigned char *m_pBufForDriver;
        unsigned char *m_pBufForSaveImage;
        int image_empty_count = 0; //空图帧数
    };
} // namespace hk_camera
#endif
