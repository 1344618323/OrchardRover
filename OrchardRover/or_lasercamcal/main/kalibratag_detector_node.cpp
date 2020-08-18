#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "PinholeCamera.h"
#include "config.h"
#include "utilities.h"
#include "calc_cam_pose.h"

template<typename T>
T readParam(ros::NodeHandle &n, std::string name) {
    std::cout << name << std::endl;
    T ans;
    if (n.getParam(name, ans)) {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    } else {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

class kalibraTagDetector {
public:
    kalibraTagDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh);

    ~kalibraTagDetector();

    void imageCb(const sensor_msgs::ImageConstPtr &msg);

    bool detect(const sensor_msgs::ImageConstPtr &msg, CamPose &T);

private:
    CamPoseEst camposecal_;
    CameraPtr cameraptr_;
    ros::NodeHandle it_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    ros::Publisher detections_pub_;
    ros::Publisher pose_pub_;
};

kalibraTagDetector::kalibraTagDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh) : it_(nh) {

    std::string config_file;
    config_file = readParam<std::string>(pnh, "config_file");

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    int tagtype;
    double tagsize = 0.055;  // default size
    double tagspace = 0.3;  // default
    int black_border;
    tagtype = static_cast<int>(fsSettings["tag_type"]);
    tagsize = static_cast<double>(fsSettings["tag_size"]);
    tagspace = static_cast<double>(fsSettings["tag_spacing"]);
    black_border = static_cast<double>(fsSettings["black_border"]);
    std::cout << "tag_size: " << tagsize << " type: " << tagtype << std::endl;

    if (1 == tagtype) {
        CalibrBoardInfo info(KALIBR_TAG_PATTERN, tagsize, tagspace, black_border);
        camposecal_ = CamPoseEst(info);
    } else if (2 == tagtype) {
        CalibrBoardInfo info(APRIL_TAG_ONE, tagsize, black_border);
        camposecal_ = CamPoseEst(info);
    } else if (3 == tagtype) {
        CalibrBoardInfo info(CHESS, tagsize);
        camposecal_ = CamPoseEst(info);
    }

    cameraptr_ = nullptr;
    PinholeCamera::Parameters paras;
    paras.readFromYamlFile(config_file);
    cameraptr_ = CameraPtr(new PinholeCamera(paras));
    ROS_INFO("LOAD PINHOLE CAMERA!");

    std::string img_topic_name = "image";
    fsSettings["img_topic_name"] >> img_topic_name;
    fsSettings.release();
    image_sub_ = it_.subscribe(img_topic_name, 10, &kalibraTagDetector::imageCb, this);
}

kalibraTagDetector::~kalibraTagDetector() {
    image_sub_.shutdown();
}

void kalibraTagDetector::imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Eigen::Matrix4d Twc;
    if (cameraptr_ != nullptr) {
        // 直接传原始图像，也可以检测二维码。里面会对坐标去畸变再计算 pose.
        cv::Mat rectified = cv_ptr->image.clone();
        camposecal_.calcCamPose(cv_ptr->header.stamp.toSec(), rectified, cameraptr_, Twc);
    } else {
        ROS_ERROR("please select camera model and write in yaml file");
    }
}

bool kalibraTagDetector::detect(const sensor_msgs::ImageConstPtr &msg, CamPose &T) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }

    Eigen::Matrix4d Twc;
    if (cameraptr_ != nullptr) {
        // 直接传原始图像，也可以检测二维码。里面会对坐标去畸变再计算 pose.
        cv::Mat rectified = cv_ptr->image.clone();
        bool success_flag = camposecal_.calcCamPose(cv_ptr->header.stamp.toSec(), rectified, cameraptr_, Twc);

        T.timestamp = cv_ptr->header.stamp.toSec();
        T.twc = Twc.block(0, 3, 3, 1);
        Eigen::Matrix3d R = Twc.block(0, 0, 3, 3);
        Eigen::Quaterniond q(R);
        T.qwc = q;
        return success_flag;
    } else {
        ROS_ERROR("please select camera model and write in yaml file");
        return false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "CamPoseEstimation");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string config_file;
    config_file = readParam<std::string>(pnh, "config_file");
    readParameters(config_file);

    rosbag::Bag bag_input;
    bag_input.open(bag_path, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(scan_topic_name);
    topics.push_back(img_topic_name);
    rosbag::View views(bag_input, rosbag::TopicQuery(topics));

    // 初始视觉 pose
    kalibraTagDetector detector(nh, pnh);
    std::vector<CamPose> tagpose;
    std::string file = savePath + "apriltag_pose.txt";

    //重新建一个文件（如果没有这两行代码，数据是在txt文件中续写的）
    std::ofstream fC(file.c_str());
    fC.close();

    std::ofstream foutC(file.c_str(), std::ios::app);
    foutC.setf(std::ios::fixed, std::ios::floatfield);
    foutC.precision(9);

    for (rosbag::MessageInstance const m: views) {
        if (m.getTopic() == img_topic_name) {
            sensor_msgs::ImageConstPtr img = m.instantiate<sensor_msgs::Image>();
            CamPose Twc;
            if (detector.detect(img, Twc)) {
                tagpose.push_back(Twc);

                EulerAngles rpy = ToEulerAngles(Twc.qwc);

                foutC << Twc.timestamp << " ";
                foutC.precision(10);
                foutC << Twc.twc(0) << " "
                      << Twc.twc(1) << " "
                      << Twc.twc(2) << " "
                      << Twc.qwc.x() << " "
                      << Twc.qwc.y() << " "
                      << Twc.qwc.z() << " "
                      << Twc.qwc.w() << " "
                      << rpy.roll << " "
                      << rpy.pitch << " "
                      << rpy.yaw
                      << std::endl;
            }
        }
        if (!ros::ok())
            break;
    }
    foutC.close();
    return 0;
}
