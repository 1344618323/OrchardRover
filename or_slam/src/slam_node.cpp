#include "slam_node.h"

//s1默认是10hz，720个点
SlamNode::SlamNode(std::string name) {
    CHECK(Init()) << "Module " << name << " initialized failed!"
                  << " pure localization is:" << pure_localization_;
}

SlamNode::~SlamNode() {
}

bool SlamNode::Init() {
    nh_.param<bool>("pure_localization", pure_localization_, false);
    nh_.param<bool>("use_sim", use_sim_, true);
    nh_.param<std::string>("odom_frame_id", odom_frame_, "odom");
    nh_.param<std::string>("base_frame_id", base_frame_, "base_link");
    nh_.param<std::string>("base_frame_id", gps_frame_, "gps_link");
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

    slam_ptr_ = std::make_unique<optimizedSlam::OptimizedSlam>(init_pose_, &nh_);

    if (use_sim_) {
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 3; j++) {
                CTrunkPoints[i * 3 + j][0] = i * 4 + 6.5;
                CTrunkPoints[i * 3 + j][1] = j * 4 + 6;
            }
        }

        // 模拟器检测树干方位
        laser_scan_sub_ = nh_.subscribe(laser_topic_, 100, &SlamNode::LaserScanCallbackForSim, this);
        // 模拟器发送树干方位
        sim_trunk_obs_pub_ = nh_.advertise<or_msgs::TrunkObsMsgXY>(trunk_obs_topic_, 1);
        ground_truth_sub_ = nh_.subscribe("base_pose_ground_truth", 1, &SlamNode::GroundTruthCallbackForSim, this);
    }

    // 处理树干方位消息
    trunk_obs_sub_ =
            std::make_unique<message_filters::Subscriber<or_msgs::TrunkObsMsgXY >>(nh_, trunk_obs_topic_, 10);
    tf_filter_ = std::make_unique<tf::MessageFilter<or_msgs::TrunkObsMsgXY >>(*trunk_obs_sub_, *tf_listener_ptr_,
                                                                              "scan", 10);
    tf_filter_->registerCallback(boost::bind(&SlamNode::TrunkObsMsgCallback, this, _1));

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

    timer_ = nh_.createTimer(ros::Duration(0.3), &SlamNode::TimerCallbackForVisualize, this);

//    std::vector<std::string> csvtopic = {"truex", "truey", "odomx", "odomy"};
    return true;
}


void SlamNode::LaserScanCallbackForSim(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg) {
    // for simulation
    or_msgs::TrunkObsMsgXY msg;
    std::vector<double> XYs;
    std::vector<int> index;
    for (int i = 0; i < 21; i++) {
        float delta_x = CTrunkPoints[i][0] - ground_truth_pose_.position.x;
        float delta_y = CTrunkPoints[i][1] - ground_truth_pose_.position.y;
        float yaw = tf::getYaw(ground_truth_pose_.orientation);
        float diff = AngleDiff<float>(atan2(delta_y, delta_x), yaw);
        if ((delta_x * delta_x + delta_y * delta_y) < 16 && diff > -M_PI / 3 && diff < M_PI / 3) {
            int ind = diff / M_PI * 180 + 134;
            XYs.push_back(laser_scan_msg->ranges[ind] * cos(diff));
            XYs.push_back(laser_scan_msg->ranges[ind] * sin(diff));
//            std::cout << *(XYs.end() - 2) << " " << *(XYs.end() - 1) << ", " << diff << " "
//                      << laser_scan_msg->ranges[ind] << std::endl;
        }
    }
    msg.header.stamp = laser_scan_msg->header.stamp;
    msg.header.frame_id = "scan";
    msg.XY = XYs;
    sim_trunk_obs_pub_.publish(msg);
}

void SlamNode::GroundTruthCallbackForSim(const nav_msgs::Odometry::ConstPtr &ground_truth_msg) {
    ground_truth_pose_ = ground_truth_msg->pose.pose;
}

void SlamNode::TrunkObsMsgCallback(const or_msgs::TrunkObsMsgXY::ConstPtr &trunk_obs_msg) {
    last_laser_msg_timestamp_ = trunk_obs_msg->header.stamp;
    Vec3d pose_in_odom;
    // base_link(0,0)在odom中坐标
    if (!GetPoseFromTf(odom_frame_, base_frame_, trunk_obs_msg->header.stamp, pose_in_odom)) {
        //LOG_ERROR << trunk_obs_msg->header.stamp;
        //LOG_ERROR << "Couldn't determine robot's pose";
        return;
    }
    //    GetTrunkPosition(trunk_obs_msg);
    // if (!pure_localization_)
    //     slam_ptr_->Update(pose_in_odom, trunk_obs_vec_, particlecloud_msg_, lmcloud_msg_);
    // else
    //     localization_ptr_->Update(pose_in_odom, trunk_obs_vec_, particlecloud_msg_);

    pose_in_odom_ = pose_in_odom;

    slam_ptr_->AddNodeData(pose_in_odom, trunk_obs_msg->XY, trunk_obs_msg->header.stamp);

    PublishTf();
}

bool SlamNode::PublishTf() {
    ros::Time transform_expiration = (last_laser_msg_timestamp_ + transform_tolerance_);
    // Subtracting base to odom from map to base and send map to odom instead
    tf::Stamped<tf::Pose> odom_to_map;
    Eigen::Vector3d global_pose = slam_ptr_->GetPose(pose_in_odom_);

    try {
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(global_pose[2]),
                             tf::Vector3(global_pose[0],
                                         global_pose[1],
                                         0.0));
        tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(),
                                             last_laser_msg_timestamp_,
                                             base_frame_);
        this->tf_listener_ptr_->transformPose(odom_frame_,
                                              tmp_tf_stamped,
                                              odom_to_map);
    }
    catch (tf::TransformException &e) {
        //LOG_ERROR << "Failed to subtract base to odom transform" << e.what();
        return false;
    }

    latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                               tf::Point(odom_to_map.getOrigin()));

    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        transform_expiration,
                                        global_frame_,
                                        odom_frame_);
    this->tf_broadcaster_ptr_->sendTransform(tmp_tf_stamped);

    // if (!pure_localization_) {
    //     csv_writer_->write(slam_ptr_->gus_pose(0));
    //     csv_writer_->write(slam_ptr_->gus_pose(1));
    // } else {
    //     csv_writer_->write(localization_ptr_->gus_pose(0));
    //     csv_writer_->write(localization_ptr_->gus_pose(1));
    // }

    // csv_writer_->write(pose_in_odom_(0));
    // csv_writer_->write(pose_in_odom_(1));

    return true;
}

void SlamNode::TimerCallbackForVisualize(const ros::TimerEvent &e) {
//    if (particlecloud_pub_.getNumSubscribers() > 0) {
//        particlecloud_msg_.header.stamp = ros::Time::now();
//        particlecloud_pub_.publish(particlecloud_msg_);
//    }
//
//    if (!pure_localization_ && lmcloud_pub_.getNumSubscribers() > 0) {
//        lmcloud_msg_.header.stamp = ros::Time::now();
//        lmcloud_pub_.publish(lmcloud_msg_);
//    }

    lmcloud_msg_.header.stamp = ros::Time::now();
    lmcloud_msg_.points.clear();
    const std::map<int, Eigen::Vector3d> global_lms = slam_ptr_->GetLandmarks();

    for (auto &lm:global_lms) {
        geometry_msgs::Point temp;
        temp.x = lm.second(0);
        temp.y = lm.second(1);
        temp.z = 0;
        lmcloud_msg_.points.push_back(temp);
    }
    lmcloud_pub_.publish(lmcloud_msg_);
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
        //LOG_ERROR << "Couldn't transform from "
//                  << source_frame
//                  << " to "
//                  << target_frame;
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

int main(int argc, char **argv) {
    // GLogWrapper glog_wrapper(argv[0]);
    ros::init(argc, argv, "slam_node");
    SlamNode slam_node("slam_node");
    ros::spin();
    return 0;
}
