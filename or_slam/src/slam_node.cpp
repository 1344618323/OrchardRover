#include "slam_node.h"

//s1默认是10hz，720个点

SlamNode::SlamNode(std::string name) {
    CHECK(Init()) << "Module " << name << " initialized failed!"
                  << " pure localization is:" << pure_localization_;
}

bool SlamNode::Init() {
    nh_.param<bool>("pure_localization", pure_localization_, false);
    nh_.param<bool>("sim_sign", sim_sign_, true);
    nh_.param<bool>("use_ultrasonic", use_ultrasonic_, false);
    nh_.param<std::string>("odom_frame_id", odom_frame_, "odom");
    nh_.param<std::string>("base_frame_id", base_frame_, "base_link");
    nh_.param<std::string>("global_frame_id", global_frame_, "map");
    nh_.param<std::string>("laser_topic_name", laser_topic_, "scan");
    nh_.param<std::string>("trunk_topic_name", trunk_obs_topic_, "trunk_obs");

    double transform_tolerance;
    nh_.param<double>("transform_tolerance", transform_tolerance, 0.1);
    this->transform_tolerance_ = ros::Duration(transform_tolerance);

    nh_.param<double>("initial_pose_x", init_pose_(0), 0);
    nh_.param<double>("initial_pose_y", init_pose_(1), 0);
    nh_.param<double>("initial_pose_a", init_pose_(2), 0);

    nh_.param<double>("initial_cov_xx", init_cov_(0), 0.1); //用于初始化高斯分布滤波器
    nh_.param<double>("initial_cov_yy", init_cov_(1), 0.1);
    nh_.param<double>("initial_cov_aa", init_cov_(2), 0.1);

    tf_listener_ptr_ = std::make_unique<tf::TransformListener>();
    tf_broadcaster_ptr_ = std::make_unique<tf::TransformBroadcaster>();

    // 用于记录雷达数据
    // laser_scan_sub_ = nh_.subscribe(laser_topic_, 100, &SlamNode::LaserScanCallbackForCheck, this);
    // std::vector<std::string> csvtopic = {"cnt", "x", "y"};
    // csvWriter_ = std::make_unique<CsvWriter>("/home/cxn/data.csv", csvtopic);

    // 积攒雷达数据
    // laser_scan_sub_ = nh_.subscribe(laser_topic_, 100, &SlamNode::LaserScanCallbackForSave, this);

    if (sim_sign_) {
        // 模拟器检测树干方位
        laser_scan_sub_ = nh_.subscribe(laser_topic_, 100, &SlamNode::LaserScanCallbackForSim, this);
        // 模拟器发送树干方位
        sim_trunk_obs_pub_ = nh_.advertise<or_msgs::TrunkObsMsg>(trunk_obs_topic_, 100);
    }

    // 处理树干方位消息
    trunk_obs_sub_ = std::make_unique<message_filters::Subscriber<or_msgs::TrunkObsMsg>>(nh_, trunk_obs_topic_, 100);
    tf_filter_ = std::make_unique<tf::MessageFilter<or_msgs::TrunkObsMsg>>(*trunk_obs_sub_, *tf_listener_ptr_,
                                                                           odom_frame_, 100);
    tf_filter_->registerCallback(boost::bind(&SlamNode::TrunkObsMsgCallback, this, _1));

    // 配置算法
    if (!pure_localization_) {
        slam_ptr_ = std::make_unique<FastSlam>(init_pose_, init_cov_, &nh_, use_ultrasonic_);
    } else {
        // 读取树干坐标
        std::vector<Vec2d> lms;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 7; j++) {
                Vec2d p;
                p << 6.5 + j * 4, 6 + i * 4;
                lms.push_back(p);
            }
        }

        localization_ptr_ = std::make_unique<FastSlamLocalization>(init_pose_, init_cov_,
                                                                   &nh_, lms, use_ultrasonic_);
    }

    // 发布点云
    particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
    particlecloud_msg_.header.frame_id = global_frame_;
    // 发布树干坐标
    lmcloud_pub_ = nh_.advertise<visualization_msgs::Marker>("lmcloud", 2);
    lmcloud_msg_.header.frame_id = global_frame_;
    lmcloud_msg_.action = visualization_msgs::Marker::ADD;
    lmcloud_msg_.type = visualization_msgs::Marker::POINTS;
    lmcloud_msg_.pose.position.x = 0;
    lmcloud_msg_.pose.position.y = 0;
    lmcloud_msg_.pose.position.z = 0;
    lmcloud_msg_.pose.orientation.x = 0.0;
    lmcloud_msg_.pose.orientation.y = 0.0;
    lmcloud_msg_.pose.orientation.z = 0.0;
    lmcloud_msg_.pose.orientation.w = 1.0;
    lmcloud_msg_.scale.x = 0.2;
    lmcloud_msg_.scale.y = 0.2;
    lmcloud_msg_.color.r = 0.0;
    lmcloud_msg_.color.g = 0.0;
    lmcloud_msg_.color.b = 1.0;
    lmcloud_msg_.color.a = 1.0; //一定要初始化，否则默认为0，看不见！
    lmcloud_msg_.ns = "lm";
    lmcloud_msg_.id = 0;


    // 计算雷达相对机器人位置
    GetLaserPose();

    return true;
}

void SlamNode::GetLaserPose() {
    if ((use_ultrasonic_ && sim_sign_) || !use_ultrasonic_) {
        auto laser_scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(laser_topic_);

        Vec3d laser_pose;
        GetPoseFromTf(base_frame_, laser_scan_msg->header.frame_id, ros::Time(), laser_pose);
        laser_pose[2] = 0; // No need for rotation, or will be error
        DLOG_INFO << "Received laser's pose wrt robot: " << laser_pose[0] << ", " << laser_pose[1] << ", "
                  << laser_pose[2];

        if(use_ultrasonic_){
            if (!pure_localization_)
                slam_ptr_->SetMultiSensorPose(laser_pose,laser_pose);
            else
                localization_ptr_->SetMultiSensorPose(laser_pose,laser_pose);
        }
        else {
            if (!pure_localization_)
                slam_ptr_->SetSensorPose(laser_pose);
            else
                localization_ptr_->SetSensorPose(laser_pose);
        }

        TransformLaserscanToBaseFrame(laser_angle_min_, laser_angle_increment_, *laser_scan_msg);
    } else {
        Vec3d sensor_pose1;
        Vec3d sensor_pose2;

        if (!pure_localization_)
            slam_ptr_->SetMultiSensorPose(sensor_pose1, sensor_pose2);
        else
            localization_ptr_->SetMultiSensorPose(sensor_pose1, sensor_pose2);
    }


    // 收集雷达数据
    //    if (laser_msg_queue_.size() >= 10)
    //        l  if (lmcloud_pub_.getNumSubscribers() > 0) {
    //                // Publish the resulting particle cloud
    //                lmcloud_msg_.points.resize(1000);
    //                for (int i = 0; i < 1000; i++) {
    //                    lmcloud_msg_.points[i].x = drand48() * 5 + 7;
    //                    lmcloud_msg_.points[i].y = drand48() * 5 + 10;
    //                    lmcloud_msg_.points[i].z = 0;
    //                }
    //                lmcloud_msg_.header.stamp = ros::Time::now();
    //                lmcloud_pub_.publish(lmcloud_msg_);
    //            }aser_msg_queue_.erase(std::begin(laser_msg_queue_));
    //    laser_msg_queue_.push_back(*laser_scan_msg);
}

void SlamNode::TrunkObsMsgCallback(const or_msgs::TrunkObsMsg::ConstPtr &trunk_obs_msg) {
    Vec3d pose_in_odom;
    // base_link(0,0)在odom中坐标
    if (!GetPoseFromTf(odom_frame_, base_frame_, trunk_obs_msg->header.stamp, pose_in_odom)) {
        LOG_ERROR << "Couldn't determine robot's pose";
        return;
    }

    //    sensor_msgs::LaserScan laser_scan_msg = ChooseLaserScan(trunk_obs_msg);//找一个时间戳最接近的
    //    GetTrunkPosition(laser_scan_msg, trunk_obs_msg);

    GetTrunkPosition(trunk_obs_msg);

    if (!pure_localization_)
        slam_ptr_->Update(pose_in_odom, trunk_obs_vec_, particlecloud_msg_, lmcloud_msg_);
    else
        localization_ptr_->Update(pose_in_odom, trunk_obs_vec_, particlecloud_msg_);

    PublishVisualize();

    PublishTf();
}

bool SlamNode::PublishTf() {
    geometry_msgs::TransformStamped gimbal_tf_;
    gimbal_tf_.header.frame_id = "map";
    gimbal_tf_.child_frame_id = "odom";

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0,
                                                                          0.0,
                                                                          0.0);
    gimbal_tf_.header.stamp = ros::Time().now();
    gimbal_tf_.transform.rotation = q;
    gimbal_tf_.transform.translation.x = init_pose_(0);
    gimbal_tf_.transform.translation.y = init_pose_(1);
    gimbal_tf_.transform.translation.z = 0.15;
    tf_broadcaster_ptr_->sendTransform(gimbal_tf_);
}

void SlamNode::TransformLaserscanToBaseFrame(double &angle_min,
                                             double &angle_increment,
                                             const sensor_msgs::LaserScan &laser_scan_msg) {
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, laser_scan_msg.angle_min);
    tf::Stamped<tf::Quaternion> min_q(q, laser_scan_msg.header.stamp,
                                      laser_scan_msg.header.frame_id);
    q.setRPY(0.0, 0.0, laser_scan_msg.angle_min + laser_scan_msg.angle_increment);
    tf::Stamped<tf::Quaternion> inc_q(q, laser_scan_msg.header.stamp,
                                      laser_scan_msg.header.frame_id);

    try {
        tf_listener_ptr_->transformQuaternion(base_frame_,
                                              min_q,
                                              min_q);
        tf_listener_ptr_->transformQuaternion(base_frame_,
                                              inc_q,
                                              inc_q);
    }
    catch (tf::TransformException &e) {
        LOG_WARNING << "Unable to transform min/max laser angles into base frame: " << e.what();
        return;
    }

    angle_min = tf::getYaw(min_q);
    angle_increment = (tf::getYaw(inc_q) - angle_min);

    // Wrapping angle to [-pi .. pi]
    angle_increment = (std::fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI);
}

void SlamNode::PublishVisualize() {
    if (particlecloud_pub_.getNumSubscribers() > 0) {
        particlecloud_msg_.header.stamp = ros::Time::now();
        particlecloud_pub_.publish(particlecloud_msg_);
    }

    if (!pure_localization_ && lmcloud_pub_.getNumSubscribers() > 0) {
        lmcloud_msg_.header.stamp = ros::Time::now();
        lmcloud_pub_.publish(lmcloud_msg_);
    }
};

void SlamNode::GetTrunkPosition(const sensor_msgs::LaserScan &laser_scan_msg,
                                const or_msgs::TrunkObsMsg::ConstPtr &trunk_obs_msg) {
    //    trunk_obs_vec_.clear();
    //    trunk_obs_vec_.shrink_to_fit();
    //    //求角度
    //    for (int i = 0; i < trunk_obs_msg->angle.size(); i + 2) {
    //        double mid = (trunk_obs_msg->angle[i] + trunk_obs_msg->angle[i + 1]) / 2;
    //        int mid_index = mid * 360 / M_PI;
    //        if (mid_index < 0)
    //            mid_index += 720;
    //        Vec2d tp;
    //        tp(0) = laser_scan_msg.ranges[mid_index];
    //        tp(1) = mid;
    //        trunk_obs_vec_.push_back(tp);
    //    }
}

void SlamNode::GetTrunkPosition(const or_msgs::TrunkObsMsg::ConstPtr &trunk_obs_msg) {
    trunk_obs_vec_.clear();
    trunk_obs_vec_.resize(trunk_obs_msg->ranges.size());
    trunk_obs_vec_.shrink_to_fit();
    for (int i = 0; i < trunk_obs_msg->ranges.size(); i++) {
        if (use_ultrasonic_) {
            if (trunk_obs_msg->valids[i] == 2) {
                trunk_obs_vec_[i](0) = trunk_obs_msg->ranges[i] + trunk_radius_avg_ +
                                       RandomGaussianNumByStdDev(trunk_radius_sigma_); //加上树干半径
                trunk_obs_vec_[i](1) = trunk_obs_msg->bearings[i];
            } else {
                trunk_obs_vec_[i](0) = -1;
            }
        } else {
            trunk_obs_vec_[i](0) = trunk_obs_msg->ranges[i] + trunk_radius_avg_ +
                                   RandomGaussianNumByStdDev(trunk_radius_sigma_); //加上树干半径
            trunk_obs_vec_[i](1) = trunk_obs_msg->bearings[i];
        }
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
        this->tf_listener_ptr_->transformPose(target_frame,
                                              ident,
                                              pose_stamp);
    }
    catch (tf::TransformException &e) {
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

sensor_msgs::LaserScan SlamNode::ChooseLaserScan(const or_msgs::TrunkObsMsg::ConstPtr &trunk_obs_msg) {
    double mintime = std::numeric_limits<float>::max();
    int minindex = 0;
    for (int j = 0; j < laser_msg_queue_.size(); j++) {
        double temp = fabs((laser_msg_queue_[j].header.stamp - trunk_obs_msg->header.stamp).toSec());
        mintime = mintime > temp ? temp : mintime;
        minindex = j;
    }
    assert(laser_msg_queue_.size() > 0);
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

void SlamNode::LaserScanCallbackForSave(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg) {
    if (laser_msg_queue_.size() >= 10)
        laser_msg_queue_.erase(std::begin(laser_msg_queue_));
    laser_msg_queue_.push_back(*laser_scan_msg);
}

void SlamNode::LaserScanCallbackForSim(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg) {
    // for simulation
    or_msgs::TrunkObsMsg msg;
    std::vector<int8_t> valid_array;
    std::vector<double> range_array;
    std::vector<double> bearing_array;
    if (use_ultrasonic_) {
        valid_array.resize(2);
        range_array.resize(2);
        bearing_array.resize(2);

        for (int i = 0; i < 2; i++) {
            valid_array[i] = 0;
        }

        int start_ind = -1, end_ind = -1;
        int valid_num = 0;

        for (int i = 1; i < laser_scan_msg->ranges.size() && valid_num < 2; i++) { ;
            if (laser_scan_msg->ranges[i - 1] > 4.9 && laser_scan_msg->ranges[i] <= 4.9 && start_ind == -1) {
                start_ind = i;
            } else if (laser_scan_msg->ranges[i - 1] <= 4.9 && laser_scan_msg->ranges[i] > 4.9 && start_ind != -1) {
                end_ind = i;
                int mid_ind = (start_ind + end_ind) / 2;
                start_ind = end_ind = -1;
                valid_array[valid_num] = 2;
                range_array[valid_num] = laser_scan_msg->ranges[mid_ind];
                bearing_array[valid_num] = laser_angle_min_ + mid_ind * laser_angle_increment_;
                valid_num++;
            }
        }

    } else {
        int start_ind = -1, end_ind = -1;
        for (int i = 1; i < laser_scan_msg->ranges.size(); i++) { ;
            if (laser_scan_msg->ranges[i - 1] > 4.9 && laser_scan_msg->ranges[i] <= 4.9 && start_ind == -1) {
                start_ind = i;
            } else if (laser_scan_msg->ranges[i - 1] <= 4.9 && laser_scan_msg->ranges[i] > 4.9 && start_ind != -1) {
                end_ind = i;
                int mid_ind = (start_ind + end_ind) / 2;
                start_ind = end_ind = -1;
                range_array.push_back(laser_scan_msg->ranges[mid_ind]);
                bearing_array.push_back(laser_angle_min_ + mid_ind * laser_angle_increment_);
            }
        }
    }
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = odom_frame_;
    msg.valids = valid_array;
    msg.ranges = range_array;
    msg.bearings = bearing_array;
    sim_trunk_obs_pub_.publish(msg);
}

int main(int argc, char **argv) {
    GLogWrapper glog_wrapper(argv[0]);
    ros::init(argc, argv, "fastslam_node");
    SlamNode fastslam_node("fastslam_node");
    ros::spin();
    return 0;
}
