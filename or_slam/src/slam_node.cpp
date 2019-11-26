#include "slam_node.h"

//s1默认是10hz，720个点

SlamNode::SlamNode(std::string name) {
    CHECK(Init()) << "Module " << name << " initialized failed!";
}

bool SlamNode::Init() {
    nh_.param<std::string>("odom_frame_id", odom_frame_, "odom");
    nh_.param<std::string>("base_frame_id", base_frame_, "base_link");
    nh_.param<std::string>("global_frame_id", global_frame_, "map");
    nh_.param<std::string>("laser_topic_name", laser_topic_, "scan");
    nh_.param<std::string>("trunk_topic_name", trunk_topic_, "trunk_angle");

    double transform_tolerance;
    nh_.param<double>("transform_tolerance", transform_tolerance, 0.1);
    this->transform_tolerance_ = ros::Duration(transform_tolerance);

    nh_.param<double>("initial_pose_x", init_pose_(0), 0);
    nh_.param<double>("initial_pose_y", init_pose_(1), 0);
    nh_.param<double>("initial_pose_a", init_pose_(2), 0);

    nh_.param<double>("initial_cov_xx", init_cov_(0), 0.1);//用于初始化高斯分布滤波器
    nh_.param<double>("initial_cov_yy", init_cov_(1), 0.1);
    nh_.param<double>("initial_cov_aa", init_cov_(2), 0.1);

    // 用于记录雷达数据
    // laser_scan_sub_ = nh_.subscribe(laser_topic_, 100, &SlamNode::LaserScanCallbackForCheck, this);
    // std::vector<std::string> csvtopic = {"cnt", "x", "y"};
    // csvWriter_ = std::make_unique<CsvWriter>("/home/cxn/data.csv", csvtopic);

    // 积攒雷达数据
//    laser_scan_sub_ = nh_.subscribe(laser_topic_, 100, &SlamNode::LaserScanCallback, this);

    trunk_angle_sub = std::make_shared<message_filters::Subscriber<or_msgs::TrunkAngleMsg>>(nh_, trunk_topic_, 100);
    tf_filter = std::make_shared<tf::MessageFilter<or_msgs::TrunkAngleMsg>>(*trunk_angle_sub, tf_listener, odom_frame_,
                                                                            100);
    tf_filter->registerCallback(boost::bind(&SlamNode::TrunkAngleMsgCallback, this, _1));

    slam_ptr_ = std::make_unique<FastSlam>(init_pose_, init_cov_, &nh_);

    GetLaserPose();

    return true;
}


void SlamNode::GetLaserPose() {
    auto laser_scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(laser_topic_);

    Vec3d laser_pose;
    laser_pose.setZero();
    GetPoseFromTf(base_frame_, laser_scan_msg->header.frame_id, ros::Time(), laser_pose);
    laser_pose[2] = 0; // No need for rotation, or will be error
    DLOG_INFO << "Received laser's pose wrt robot: "<<
              laser_pose[0] << ", " <<
              laser_pose[1] << ", " <<
              laser_pose[2];

    slam_ptr_->SetSensorPose(laser_pose);

    if (laser_msg_queue_.size() >= 10)
        laser_msg_queue_.erase(std::begin(laser_msg_queue_));
    laser_msg_queue_.push_back(*laser_scan_msg);
}

void SlamNode::TrunkAngleMsgCallback(const or_msgs::TrunkAngleMsg::ConstPtr &trunk_angle_msg) {
    Vec3d pose_in_odom;//base_link(0,0)在odom中坐标
    if (!GetPoseFromTf(odom_frame_, base_frame_, trunk_angle_msg->header.stamp, pose_in_odom)) {
        LOG_ERROR << "Couldn't determine robot's pose";
        return;
    }

    sensor_msgs::LaserScan laser_scan_msg = ChooseLaserScan(trunk_angle_msg);//找一个时间戳最接近的

    GetTrunkPosition(laser_scan_msg, trunk_angle_msg);

    slam_ptr_->Update(pose_in_odom, trunk_pos_vec_);
}


bool SlamNode::GetTrunkPosition(const sensor_msgs::LaserScan &laser_scan_msg,
                                const or_msgs::TrunkAngleMsg::ConstPtr &trunk_angle_msg) {
    trunk_pos_vec_.clear();
    trunk_pos_vec_.shrink_to_fit();
    //求角度
    for (int i = 0; i < trunk_angle_msg->angle.size(); i + 2) {
        double mid = (trunk_angle_msg->angle[i] + trunk_angle_msg->angle[i + 1]) / 2;
        int midindex = mid * 360 / M_PI;
        if (midindex < 0)
            midindex += 720;
        Vec2d tp;
        tp(0) = laser_scan_msg.ranges[midindex];
        tp(1) = mid;
        trunk_pos_vec_.push_back(tp);
    }
}


bool SlamNode::GetPoseFromTf(const std::string &target_frame,
                             const std::string &source_frame,
                             const ros::Time &timestamp,
                             Vec3d &pose) {
    tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
                                              tf::Vector3(0, 0, 0)),
                                timestamp,
                                source_frame);
    //求source_frame中的原点位姿在target_frame中的位姿
    tf::Stamped<tf::Pose> pose_stamp;
    try {
        this->tf_listener.transformPose(target_frame,
                                        ident,
                                        pose_stamp);
    } catch (tf::TransformException &e) {
        LOG_ERROR << "Couldn't transform from "
                  << source_frame
                  << "to "
                  << target_frame;
        return false;
    }

    pose[0] = pose_stamp.getOrigin().x();
    pose[1] = pose_stamp.getOrigin().y();
    pose[2] = 0;
    double yaw, pitch, roll;
    pose_stamp.getBasis().getEulerYPR(yaw, pitch, roll);
    pose[2] = yaw;
    return true;
}


void SlamNode::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg) {
    if (laser_msg_queue_.size() >= 10)
        laser_msg_queue_.erase(std::begin(laser_msg_queue_));
    laser_msg_queue_.push_back(*laser_scan_msg);
}

sensor_msgs::LaserScan SlamNode::ChooseLaserScan(const or_msgs::TrunkAngleMsg::ConstPtr &trunk_angle_msg) {
    double mintime = std::numeric_limits<float>::max();
    int minindex = 0;
    for (int j = 0; j < laser_msg_queue_.size(); j++) {
        double temp = fabs((laser_msg_queue_[j].header.stamp - trunk_angle_msg->header.stamp).toSec());
        mintime = mintime > temp ? temp : mintime;
        minindex = j;
    }
    assert(laser_msg_queue_.size()>0);
    return laser_msg_queue_[minindex];
}


void SlamNode::LaserScanCallbackForCheck(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg) {
    if (csvWriter_ != nullptr) {
        static int64_t cnt;
        for (int i = 0; i < laser_scan_msg->ranges.size(); i++) {
            csvWriter_->write(cnt);
            if (laser_scan_msg->ranges[i] > 0.15 && laser_scan_msg->ranges[i] < 40.0) {
                csvWriter_->write(laser_scan_msg->ranges[i] * cos(i * M_PI / 360));
                csvWriter_->write(laser_scan_msg->ranges[i] * sin(i * M_PI / 360));
            } else {
                csvWriter_->write(0.0);
                csvWriter_->write(0.0);
            }
        }
        cnt++;
    }
}

int main(int argc, char **argv) {
    GLogWrapper glog_wrapper(argv[0]);
    ros::init(argc, argv, "fastslam_node");
    SlamNode fastslam_node("fastslam_node");
    ros::spin();
    return 0;
}
