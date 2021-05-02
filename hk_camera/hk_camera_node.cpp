#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "hk_camera.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hk_cam_hk_cam_node");
    ros::NodeHandle nh;
    bool visualize;
    nh.param("hk_camera/Visualize", visualize, false);

    hk_camera::HkCamera camera(nh);

    std::shared_ptr<image_transport::ImageTransport> cam_img_trans =
        std::make_shared<image_transport::ImageTransport>(nh);
    image_transport::Publisher image_pub =
        cam_img_trans->advertise("/usb_cam/image_raw", 1);

    ros::Rate loop_rate(40);
    int i = 0;
    while (ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
        cv::Mat frame;

        //        std::chrono::steady_clock::time_point t1 =
        //        std::chrono::steady_clock::now();
        camera >> frame;
        if (frame.empty())
            continue;
        // cv::imwrite("/home/cxn/img/" + std::to_string(i++) + ".png", frame);
        // cv::waitKey();

        cv::resize(frame, frame, cv::Size(0, 0), 0.5, 0.5);
        // std::cout << "frame.rows" << frame.rows <<" frame.cols: " <<
        // frame.cols << std::endl;

        //        std::chrono::steady_clock::time_point t2 =
        //        std::chrono::steady_clock::now();
        //        std::chrono::duration<double> time_used =
        //        std::chrono::duration_cast<std::chrono::duration<double>>(
        //                t2 - t1);
        //        std::cout << "Get image time cost = " << time_used.count() <<
        //        " seconds. " << std::endl;

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        image_pub.publish(msg);

        //        double time = ((double) cv::getTickCount() - start) /
        //        cv::getTickFrequency(); std::cout << "HK_camera,Time:" << time
        //        << "\tFPS:" << 1 / time << std::endl;

        if (visualize) {
            cv::imshow("HK vision", frame);
            cv::waitKey(10);
        }
    }
    return 0;
}
