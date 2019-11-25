#include "leonard_hk_camera.hpp"

Camera::Camera() : handle(NULL)
{
    cam_img_trans = std::make_shared<image_transport::ImageTransport>(hk_cam_node);
    image_pub = cam_img_trans->advertise("hk_image", 1);

    hk_cam_node.param("camera/width", width, 1440);
    hk_cam_node.param("camera/height", height, 1080);
    hk_cam_node.param("camera/FrameRateEnable", FrameRateEnable, (int)true);
    hk_cam_node.param("camera/FrameRate", FrameRate, 100);
    hk_cam_node.param("camera/ExposureTime", ExposureTime, 1000);
    hk_cam_node.param("camera/GammaEnable", GammaEnable, (int)true);
    hk_cam_node.param("camera/Gamma", Gamma, (float)0.7);
    hk_cam_node.param("camera/Gain", Gain, (float)5);
    hk_cam_node.param("camera/SaturationEnable", SaturationEnable, (int)true);
    hk_cam_node.param("camera/Saturation", Saturation, 128);

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
        exit(-1);
    }
    unsigned int nIndex = 0;
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                break;
            }
            PrintDeviceInfo(pDeviceInfo);
        }
    }
    else
    {
        printf("Find No Devices!\n");
        exit(-1);
    }

    // 选择设备并创建句柄
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        exit(-1);
    }

    // 打开设备
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
        exit(-1);
    }

    this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
    this->set(CAP_PROP_FRAMERATE, FrameRate);
    this->set(CAP_PROP_HEIGHT, height);
    this->set(CAP_PROP_WIDTH, width);
    this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime);
    this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable);
    this->set(CAP_PROP_GAMMA, Gamma);
    this->set(CAP_PROP_GAIN, Gain);
    nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 0); //白平衡 非自适应（给定参数0）
    int rgb[3] = {1742, 1024, 2371};                          //白平衡度
    for (int i = 0; i < 3; i++)
    {
        nRet = MV_CC_SetEnumValue(handle, "BalanceRatioSelector", i);
        nRet = MV_CC_SetIntValue(handle, "BalanceRatio", rgb[i]);
    }
    if (MV_OK == nRet)
    {
        printf("set BalanceRatio OK!\n");
    }
    else
    {
        printf("Set BalanceRatio Failed! nRet = [%x]\n\n", nRet);
    }
    this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable);
    this->set(CAP_PROP_SATURATION, Saturation);
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0); //软件触发
    if (MV_OK == nRet)
    {
        printf("set TriggerMode OK!\n");
    }
    else
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
    }
    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014); //图像格式 目前 RGB
    if (MV_OK == nRet)
    {
        printf("set PixelFormat OK!\n");
    }
    else
    {
        printf("MV_CC_SetPixelFormat fail! nRet [%x]\n", nRet);
    }
    MVCC_ENUMVALUE t = {0};
    nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &t);
    if (MV_OK == nRet)
    {
        printf("PixelFormat :%d!\n", t.nCurValue);
    }
    else
    {
        printf("get PixelFormat fail! nRet [%x]\n", nRet);
    }
    // 开始取流
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
        exit(-1);
    }

    this->HKWorkThread(handle);
}

Camera::~Camera()
{
    int nRet;
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
        exit(-1);
    }
    printf("MV_CC_StopGrabbing succeed.\n");
    // 关闭设备
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
        exit(-1);
    }
    printf("MV_CC_CloseDevice succeed.\n");
    // 销毁句柄
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
        exit(-1);
    }
    printf("MV_CC_DestroyHandle succeed.\n");
}

bool Camera::set(CamerProperties type, float value)
{
    switch (type)
    {
    case CAP_PROP_FRAMERATE_ENABLE:
    {
        nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", value);
        if (MV_OK == nRet)
        {
            printf("set AcquisitionFrameRateEnable OK!\n");
        }
        else
        {
            printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n\n", nRet);
        }
        break;
    }
    case CAP_PROP_FRAMERATE:
    {
        nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", value);
        if (MV_OK == nRet)
        {
            printf("set AcquisitionFrameRate OK!\n");
        }
        else
        {
            printf("Set AcquisitionFrameRate Failed! nRet = [%x]\n\n", nRet);
        }
        break;
    }
    case CAP_PROP_HEIGHT:
    {
        nRet = MV_CC_SetIntValue(handle, "Height", value); //图像高度
        if (MV_OK == nRet)
        {
            printf("set Height OK!\n");
        }
        else
        {
            printf("Set Height Failed! nRet = [%x]\n\n", nRet);
        }
        break;
    }
    case CAP_PROP_WIDTH:
    {
        nRet = MV_CC_SetIntValue(handle, "Width", value); //图像宽度
        if (MV_OK == nRet)
        {
            printf("set Width OK!\n");
        }
        else
        {
            printf("Set Width Failed! nRet = [%x]\n\n", nRet);
        }
        break;
    }
    case CAP_PROP_EXPOSURE_TIME:
    {
        nRet = MV_CC_SetFloatValue(handle, "ExposureTime", value); //曝光时间
        if (MV_OK == nRet)
        {
            printf("set ExposureTime OK!\n");
        }
        else
        {
            printf("Set ExposureTime Failed! nRet = [%x]\n\n", nRet);
        }
        break;
    }
    case CAP_PROP_GAMMA_ENABLE:
    {
        nRet = MV_CC_SetBoolValue(handle, "GammaEnable", value); //伽马因子是否可调  默认不可调（false）
        if (MV_OK == nRet)
        {
            printf("set GammaEnable OK!\n");
        }
        else
        {
            printf("Set GammaEnable Failed! nRet = [%x]\n\n", nRet);
        }
        break;
    }
    case CAP_PROP_GAMMA:
    {
        nRet = MV_CC_SetFloatValue(handle, "Gamma", value); //伽马越小 亮度越大
        if (MV_OK == nRet)
        {
            printf("set Gamma OK!\n");
        }
        else
        {
            printf("Set Gamma Failed! nRet = [%x]\n\n", nRet);
        }
        break;
    }
    case CAP_PROP_GAIN:
    {
        nRet = MV_CC_SetFloatValue(handle, "Gain", value); //亮度 越大越亮
        if (MV_OK == nRet)
        {
            printf("set Gain OK!\n");
        }
        else
        {
            printf("Set Gain Failed! nRet = [%x]\n\n", nRet);
        }
        break;
    }
    case CAP_PROP_SATURATION_ENABLE:
    {
        nRet = MV_CC_SetBoolValue(handle, "SaturationEnable", value); //饱和度是否可调 默认不可调(false)
        if (MV_OK == nRet)
        {
            printf("set SaturationEnable OK!\n");
        }
        else
        {
            printf("Set SaturationEnable Failed! nRet = [%x]\n\n", nRet);
        }
        break;
    }
    case CAP_PROP_SATURATION:
    {
        nRet = MV_CC_SetIntValue(handle, "Saturation", value); //饱和度 默认128 最大255
        if (MV_OK == nRet)
        {
            printf("set Saturation OK!\n");
        }
        else
        {
            printf("Set Saturation Failed! nRet = [%x]\n\n", nRet);
        }
        break;
    }
    default:
        return 0;
    }
    return nRet;
}

bool Camera::reset()
{
    nRet = this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
    nRet = this->set(CAP_PROP_FRAMERATE, FrameRate) || nRet;
    nRet = this->set(CAP_PROP_HEIGHT, height) || nRet;
    nRet = this->set(CAP_PROP_WIDTH, width) || nRet;
    nRet = this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime) || nRet;
    nRet = this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable) || nRet;
    nRet = this->set(CAP_PROP_GAMMA, Gamma) || nRet;
    nRet = this->set(CAP_PROP_GAIN, Gain) || nRet;
    nRet = this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable) || nRet;
    nRet = this->set(CAP_PROP_SATURATION, Saturation) || nRet;
    return nRet;
}

void *Camera::HKWorkThread(void *p_handle)
{
    double start;
    int nRet;
    unsigned char *m_pBufForDriver = (unsigned char *)malloc(sizeof(unsigned char) * IMAGE_DATA_SIZE);
    unsigned char *m_pBufForSaveImage = (unsigned char *)malloc(IMAGE_SAVE_SIZE);
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
    cv::Mat tmp;
    int image_empty_count = 0; //空图帧数
    while (ros::ok())
    {
        start = static_cast<double>(cv::getTickCount());
        nRet = MV_CC_GetOneFrameTimeout(p_handle, m_pBufForDriver, IMAGE_DATA_SIZE, &stImageInfo, 15);
        if (nRet != MV_OK)
        {
            if (++image_empty_count > 100)
            {
                ROS_INFO("The Number of Faild Reading Exceed The Set Value!\n");
                exit(-1);
            }
            continue;
        }
        image_empty_count = 0; //空图帧数
        //转换图像格式为BGR8

        stConvertParam.nWidth = width;                //ch:图像宽 | en:image width
        stConvertParam.nHeight = height;              //ch:图像高 | en:image height
        stConvertParam.pSrcData = m_pBufForDriver;    //ch:输入数据缓存 | en:input data buffer
        stConvertParam.nSrcDataLen = IMAGE_DATA_SIZE; //ch:输入数据大小 | en:input data size
        //stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; //ch:输出像素格式 | en:output pixel format  适用于OPENCV的图像格式
        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; //ch:输出像素格式 | en:output pixel format
        stConvertParam.pDstBuffer = m_pBufForSaveImage;             //ch:输出数据缓存 | en:output data buffer
        stConvertParam.nDstBufferSize = IMAGE_SAVE_SIZE;            //ch:输出缓存大小 | en:output buffer size
        stConvertParam.enSrcPixelType = stImageInfo.enPixelType;    //ch:输入像素格式 | en:input pixel format
        MV_CC_ConvertPixelType(p_handle, &stConvertParam);

        frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        image_pub.publish(msg);

        double time = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
        //        cout << "HK_camera,Time:" << time  << "\tFPS:" << 1 / time << endl;
        cv::imshow("HK vision", frame);
        cv::waitKey(10);
    }
    free(m_pBufForDriver);
    free(m_pBufForSaveImage);

    return 0;
}

bool Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        // 打印当前相机ip和用户自定义名字
        printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                 //当前IP
        printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); //用户定义名
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hk_cam_hk_cam_node");
    Camera hk_camera;
    return 0;
}
