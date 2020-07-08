#include "slam_node.h"

//s1默认是10hz，720个点
// 第一个数据点是：商标处，让后从上看是逆时针0~719
// 而laser-link的x轴是冲着线的方向（也就是商标处方向的反方向）：也就是说按 laser-link坐标系，数据是 -pai～pai 采集的

SlamNode::SlamNode(std::string name) {
    CHECK(Init()) << "Module " << name << " initialized failed!"
                  << " pure localization is:" << pure_localization_;
}

SlamNode::~SlamNode() {
    const std::map<int, Eigen::Vector2d> &global_lms = slam_ptr_->GetLandmarks();
    auto it = global_lms.begin();
    int idx = 0;
    while (it != global_lms.end()) {
        std::cout << "LM " << idx << ": " << it->second(0) << "," << it->second(1) << std::endl;
        it++;
        idx++;
    }
    SaveMaptoTxt(out_map_file_name_, global_lms);
}

void SlamNode::LoadMapFromTxt(std::string filename, std::map<int, Eigen::Vector2d> &lms) {
    std::ifstream f;
    f.open(filename.c_str());
    if (!f.is_open()) {
        std::cerr << " can't open map file " << std::endl;
        return;
    } else {
        std::cout << " Load map from: " << filename << std::endl;
    }

    while (!f.eof()) {
        std::string s;
        std::getline(f, s);
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            double x, y;
            int idx;
            ss >> x;
            ss >> y;
            ss >> idx;
            lms[idx] = Eigen::Vector2d(x, y);
        }
    }
}

void SlamNode::SaveMaptoTxt(std::string filename, const std::map<int, Eigen::Vector2d> &lms) {
    std::ofstream foutC;
    foutC.open(filename.c_str());
    if (!foutC.is_open()) {
        std::cerr << " can't open cam pose file " << std::endl;
        return;
    }
    std::cout << " Save map to: " << filename << std::endl;
    auto it = lms.begin();
    int idx = 0;
    while (it != lms.end()) {
        foutC.setf(std::ios::fixed, std::ios::floatfield);
        foutC.precision(5);
        foutC << it->second(0) << " "
              << it->second(1) << " "
              << idx << std::endl;
        it++;
        idx++;
    }
    foutC.close();
}


bool SlamNode::Init() {
    nh_.param<double>("or_slam/trunk_std_radius", trunk_std_radius_, 0.2);
    nh_.param<bool>("or_slam/pure_localization", pure_localization_, false);
    nh_.param<bool>("or_slam/use_sim", use_sim_, true);
    nh_.param<std::string>("or_slam/odom_frame_id", odom_frame_, "odom");
    nh_.param<std::string>("or_slam/base_frame_id", base_frame_, "base_link");
    nh_.param<std::string>("or_slam/base_frame_id", gps_frame_, "gps_link");
    nh_.param<std::string>("or_slam/global_frame_id", global_frame_, "map");
    nh_.param<std::string>("or_slam/laser_topic_name", laser_topic_, "scan");
    nh_.param<std::string>("or_slam/trunk_topic_name", trunk_obs_topic_, "trunk_obs");

    double transform_tolerance;
    nh_.param<double>("or_slam/transform_tolerance", transform_tolerance, 0.1);
    this->transform_tolerance_ = ros::Duration(transform_tolerance);

    nh_.param<double>("or_slam/initial_pose_x", init_pose_(0), 0);
    nh_.param<double>("or_slam/initial_pose_y", init_pose_(1), 0);
    nh_.param<double>("or_slam/initial_pose_a", init_pose_(2), 0);

    tf_listener_ptr_ = std::make_unique<tf::TransformListener>();
    tf_broadcaster_ptr_ = std::make_unique<tf::TransformBroadcaster>();
    slam_ptr_ = std::make_unique<optimized_slam::OptimizedSlam>(init_pose_, &nh_, pure_localization_);

    nh_.param<std::string>("or_slam/out_map_file_name", out_map_file_name_, "out_map.txt");
    nh_.param<std::string>("or_slam/read_map_file_name", read_map_file_name_, "read_map.txt");

    if (use_sim_) {
        //仿真器图的树半径是0.2m
        trunk_std_radius_ = 0.2;

        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < 4; j++) {
                C_trunkpoints_for_sim_.push_back(Eigen::Vector2d(i * 3 + 6.5, j * 3 + 5));
            }
        }

        // 模拟器检测树干方位
        laser_scan_sub_ = nh_.subscribe(laser_topic_, 10, &SlamNode::LaserScanCallbackForSim, this);
        // 模拟器发送树干方位
        trunk_obs_pub_ = nh_.advertise<or_msgs::TrunkObsMsgXY>(trunk_obs_topic_, 1);
        // 接收stage发布的全局信息
        ground_truth_sub_ = nh_.subscribe("base_pose_ground_truth", 1, &SlamNode::GroundTruthCallbackForSim, this);
        // 用于模拟器给odom加入噪声（目前配置的ros_stage发布的odom是没有噪声，绝对准确的，只能出此下策，还不会配置有噪声的odom）
        double alpha1, alpha2, alpha3, alpha4;
        nh_.param<double>("or_slam/sim_sensor_odom/alpha1", alpha1, 0.2);
        nh_.param<double>("or_slam/sim_sensor_odom/alpha2", alpha2, 0.2);
        nh_.param<double>("or_slam/sim_sensor_odom/alpha3", alpha3, 0.2);
        nh_.param<double>("or_slam/sim_sensor_odom/alpha4", alpha4, 0.2);
        sim_odom_data_generator_ptr_ = std::make_unique<SimOdomDataGenerator>(alpha1, alpha2, alpha3, alpha4);
    }

    if (pure_localization_) {
        std::map<int, Eigen::Vector2d> landmarks;
        if (use_sim_) {
            for (int i = 0; i < C_trunkpoints_for_sim_.size(); i++) {
                landmarks[i] = C_trunkpoints_for_sim_[i];
            }
        } else {
            LoadMapFromTxt(read_map_file_name_, landmarks);
        }
        slam_ptr_->SetConstantLandmarks(landmarks);
    }

    // 处理树干方位消息
    trunk_obs_sub_ =
            std::make_unique<message_filters::Subscriber<or_msgs::TrunkObsMsgXY >>(nh_, trunk_obs_topic_, 5);
    tf_filter_ = std::make_unique<tf::MessageFilter<or_msgs::TrunkObsMsgXY >>(*trunk_obs_sub_, *tf_listener_ptr_,
                                                                              "laser", 5);
    tf_filter_->registerCallback(boost::bind(&SlamNode::TrunkObsMsgXYCallback, this, _1));

    // 发布树干坐标
    landmarks_pub_ = nh_.advertise<visualization_msgs::Marker>("landmarks", 5);

    landmarks_msg_.header.frame_id = global_frame_;
    landmarks_msg_.action = visualization_msgs::Marker::ADD;
    landmarks_msg_.type = visualization_msgs::Marker::POINTS;
    landmarks_msg_.pose.position.x = 0;
    landmarks_msg_.pose.position.y = 0;
    landmarks_msg_.pose.position.z = 0;
    landmarks_msg_.pose.orientation.x = 0.0;
    landmarks_msg_.pose.orientation.y = 0.0;
    landmarks_msg_.pose.orientation.z = 0.0;
    landmarks_msg_.pose.orientation.w = 1.0;
    landmarks_msg_.scale.x = 0.2;
    landmarks_msg_.scale.y = 0.2;
    landmarks_msg_.color.r = 0.0;
    landmarks_msg_.color.g = 0.0;
    landmarks_msg_.color.b = 1.0;
    landmarks_msg_.color.a = 1.0; //一定要初始化，否则默认为0，看不见！
    landmarks_msg_.ns = "landmark";
    landmarks_msg_.id = 0;

    node_residual_line_.header.frame_id = global_frame_;
    node_residual_line_.ns = "residual";
    node_residual_line_.action = visualization_msgs::Marker::ADD;
    node_residual_line_.pose.orientation.w = 1.0;
    node_residual_line_.id = 0;
    node_residual_line_.type = visualization_msgs::Marker::LINE_LIST;
    node_residual_line_.scale.x = 0.01;
    node_residual_line_.color.r = 1.0;
    node_residual_line_.color.a = 1.0;
    node_residual_line_.lifetime = ros::Duration();

    lm_residual_line_ = node_residual_line_;
    lm_residual_line_.id = 1;
    lm_residual_line_.color.r = 1.0;
    lm_residual_line_.color.g = 1.0;
    lm_residual_line_.color.a = 1.0;

    visualize_timer_ = nh_.createTimer(ros::Duration(0.5), &SlamNode::TimerCallbackForVisualize, this);

    return GetSensorPose();
}


bool SlamNode::GetSensorPose() {
    //获得Tbase_laser
    auto laser_scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(laser_topic_);
    Eigen::Vector3d pose;
    GetPoseFromTf(base_frame_, laser_scan_msg->header.frame_id, ros::Time(), pose);
    std::cout << "Sensor Pose:" << pose.transpose() << std::endl;
    sensor_pose_.setIdentity();
    sensor_pose_.rotate(Eigen::Rotation2Dd(pose[2]));
    Eigen::Vector2d t = pose.head(2);
    sensor_pose_.pretranslate(t);
    return true;
}


void SlamNode::LaserScanCallbackForSim(const sensor_msgs::LaserScan::ConstPtr &laser_scan_msg) {
    // for simulation
    or_msgs::TrunkObsMsgXY msg;
    std::vector<geometry_msgs::Point> XYs;
    std::vector<int> index;
    for (int i = 0; i < C_trunkpoints_for_sim_.size(); i++) {
        float delta_x = C_trunkpoints_for_sim_[i][0] - ground_truth_pose_.position.x;
        float delta_y = C_trunkpoints_for_sim_[i][1] - ground_truth_pose_.position.y;
        float yaw = tf::getYaw(ground_truth_pose_.orientation);
        float diff = AngleDiff<float>(atan2(delta_y, delta_x), yaw);
        if ((delta_x * delta_x + delta_y * delta_y) < 16 && diff > -M_PI / 3 && diff < M_PI / 3) {
            int ind = diff / M_PI * 180 + 134;
            //若激光距离与到树干的距离不一致，就认为是障碍物
            if (fabs(laser_scan_msg->ranges[ind] - std::sqrt(delta_x * delta_x + delta_y * delta_y)) < 0.5) {
                //因为画的图中树的半径是0.2m
                geometry_msgs::Point p;
                p.x = (laser_scan_msg->ranges[ind] + trunk_std_radius_) * cos(diff);
                p.y = (laser_scan_msg->ranges[ind] + trunk_std_radius_) * sin(diff);
                p.z = 0.0;
                XYs.push_back(p);
            }
        }
    }
    msg.header.stamp = laser_scan_msg->header.stamp;
    msg.header.frame_id = "laser";
    msg.XYs = XYs;
    trunk_obs_pub_.publish(msg);
}

void SlamNode::GroundTruthCallbackForSim(const nav_msgs::Odometry::ConstPtr &ground_truth_msg) {
    ground_truth_pose_ = ground_truth_msg->pose.pose;
}

void SlamNode::TrunkObsMsgXYCallback(const or_msgs::TrunkObsMsgXY::ConstPtr &trunk_obs_msg) {
    last_laser_msg_timestamp_ = trunk_obs_msg->header.stamp;
    Vec3d pose_in_odom;
    // base_link(0,0)在odom中坐标
    if (!GetPoseFromTf(odom_frame_, base_frame_, trunk_obs_msg->header.stamp, pose_in_odom)) {
        //LOG(ERROR) << trunk_obs_msg->header.stamp;
        //LOG(ERROR) << "Couldn't determine robot's pose";
        return;
    }

    if (use_sim_) {
        //考虑到stage中返回的位姿是非常正确的，我们人为给他加误差
        pose_in_odom = sim_odom_data_generator_ptr_->UpdateAction2(pose_in_odom);
    }

    pose_in_odom_ = pose_in_odom;

    std::vector<Eigen::Vector2d> xys;
    for (auto &item :trunk_obs_msg->XYs) {
        xys.push_back(sensor_pose_ * Eigen::Vector2d(item.x, item.y));
    }
    slam_ptr_->AddNodeData(pose_in_odom_, xys, trunk_obs_msg->header.stamp);
    PublishTf();
}

bool SlamNode::PublishTf() {
    ros::Time transform_expiration = (last_laser_msg_timestamp_ + transform_tolerance_);
    // Subtracting base to odom from map to base and send map to odom instead
    tf::Stamped<tf::Pose> odom_to_map;
    Eigen::Vector3d global_pose = slam_ptr_->GetPose(pose_in_odom_);

    try {
        //得到了Tglobal_base
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(global_pose[2]),
                             tf::Vector3(global_pose[0],
                                         global_pose[1],
                                         0.0));
        //转成Tbase_global
        tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(),
                                             last_laser_msg_timestamp_,
                                             base_frame_);
        //转成Todom_global
        this->tf_listener_ptr_->transformPose(odom_frame_,
                                              tmp_tf_stamped,
                                              odom_to_map);
    }
    catch (tf::TransformException &e) {
        //LOG(ERROR) << "Failed to subtract base to odom transform" << e.what();
        return false;
    }

    tf::Transform global_to_odom_tf = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                                    tf::Point(odom_to_map.getOrigin()));
    //发布Tglobal_odom
    tf::StampedTransform tmp_tf_stamped(global_to_odom_tf.inverse(),
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

    landmarks_msg_.header.stamp = ros::Time::now();
    landmarks_msg_.points.clear();
    const std::map<int, Eigen::Vector2d> global_lms = slam_ptr_->GetLandmarks();

    for (auto &lm:global_lms) {
        geometry_msgs::Point temp;
        temp.x = lm.second(0);
        temp.y = lm.second(1);
        temp.z = 0;
        landmarks_msg_.points.push_back(temp);
    }
    landmarks_pub_.publish(landmarks_msg_);

    std::vector<optimized_slam::ResidualForVisualize> residual_tmp = slam_ptr_->GetResidualForVisualize();
    if (residual_tmp.size() > 0) {
        visualization_msgs::Marker node_residual = node_residual_line_;
        node_residual.header.stamp = ros::Time::now();
        visualization_msgs::Marker lm_residual = lm_residual_line_;
        lm_residual.header.stamp = ros::Time::now();

        for (auto &item:residual_tmp) {
            geometry_msgs::Point p;
            p.z = 0;

            p.x = item.node_pose.x();
            p.y = item.node_pose.y();
            node_residual.points.push_back(p);
            p.x = item.lm_obs_xy.x();
            p.y = item.lm_obs_xy.y();
            node_residual.points.push_back(p);

            lm_residual.points.push_back(p);
            p.x = item.true_obs_xy.x();
            p.y = item.true_obs_xy.y();
            lm_residual.points.push_back(p);
        }

        landmarks_pub_.publish(node_residual);
        landmarks_pub_.publish(lm_residual);
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
        //LOG(ERROR) << "Couldn't transform from "
//                  << source_frame
//                  << " to "
//                  << target_frame;
        return false;
    }

    pose[0] = pose_stamp.getOrigin().x();
    pose[1] = pose_stamp.getOrigin().y();
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

/***********************果园地图***********************/
//int main() {
//    Mat area(400, 740, CV_8UC1, Scalar(255));
//    int x_off = 130;
//    int y_off = 120;
//    for (int i = 0; i < 7; i++) {
//        for (int j = 0; j < 3; j++) {
//            circle(area, Point(i * 80 + x_off, j * 80 + y_off), 4, Scalar(0), -1);
//        }
//    }
//    rectangle(area, Rect(0, 0, 740, 400), Scalar(0), 1);
//    imshow("orchard", area);
//    imwrite("orchard.pgm", area);
//    cv::waitKey();
//    return 0;
//}

//LM 0: 7.1164,8.80264
//LM 1: 7.89926,11.7773
//LM 2: 10.6269,10.3439
//LM 3: 11.5672,13.1376
//LM 4: 14.4997,12.486
//LM 5: 13.5102,9.64658
//LM 7: 12.5932,6.74735
//LM 8: 9.5606,14.5453
//LM 9: 9.8069,7.51215
//LM 11: 15.4516,5.9673
//LM 12: 16.4092,8.84701
//LM 13: 18.4212,5.2118
//LM 14: 19.289,8.1258
//LM 15: 14.4019,3.11448
//LM 18: 8.57602,4.77747
//LM 19: 5.79729,6.12181
//LM 20: 17.2967,11.7262
//LM 21: 20.0952,11.0159
//LM 22: 21.3249,4.5065
//LM 23: 22.2556,7.41655
//LM 24: 25.1387,6.49745
//LM 25: 24.1682,3.65043
//LM 26: 22.9464,10.2034
//LM 27: 11.5225,3.89932
//LM 28: 17.3048,2.37728
//LM 29: 20.3269,1.62803
//LM 30: 23.2277,0.772609
//LM 31: 9.70113,7.58184
//LM 32: 3.31828,6.02487

//LM 0: 6.48971,7.71136
//LM 1: 6.51557,4.72049
//LM 2: 9.5004,7.6019
//LM 3: 9.42711,4.58894
//LM 4: 12.3852,4.6121
//LM 5: 12.4341,7.63565
//LM 6: 15.3754,4.55501
//LM 7: 15.4511,7.56979
//LM 8: 18.3913,4.63267
//LM 9: 18.4197,7.64599
//LM 10: 21.3952,4.63682
//LM 11: 21.3932,7.66796
//LM 13: 24.5718,10.6763
//LM 14: 24.3903,4.60888
//LM 15: 24.4003,7.6429
//LM 16: 21.5391,10.7096
//LM 17: 18.5938,10.938
//LM 18: 15.5436,10.6915
//LM 19: 12.3126,10.6401
//LM 20: 9.49539,10.6225
//LM 21: 6.54979,10.7426
//LM 22: 6.54465,13.7616
//LM 23: 9.56029,13.6675
//LM 24: 12.5647,13.6694
//LM 25: 15.6002,13.7607
//LM 26: 18.5292,13.9812
//LM 27: 21.7621,13.7351
//LM 28: 24.6453,13.7544