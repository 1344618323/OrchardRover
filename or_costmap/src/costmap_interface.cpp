//
// Created by cxn on 2020/7/2.
//

#include "costmap_interface.h"
#include "costmap_parameter_setting.pb.h"

namespace or_costmap {
    CostmapInterface::CostmapInterface(std::string map_name,
                                       tf::TransformListener &tf,
                                       std::string config_file) :
            layered_costmap_(nullptr),
            name_(map_name),
            tf_(tf),
            config_file_(config_file),
            stop_updates_(false),
            initialized_(true),
            stopped_(false),
            robot_stopped_(false),
            map_update_thread_(NULL),
            last_publish_(0),
            dist_behind_robot_threshold_to_care_obstacles_(0.05),
            map_update_thread_shutdown_(false) {

        ros::NodeHandle private_nh(map_name);
        LoadParameter();
        layered_costmap_ = new LayeredCostmap(global_frame_, is_rolling_window_, is_track_unknown_);

        //等待Tglobal_base
        std::string tf_error;
        ros::Time last_error = ros::Time::now();
        while (ros::ok() && !tf_.waitForTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1),
                                                  ros::Duration(0.01), &tf_error)) {
            ros::spinOnce();
            if (last_error + ros::Duration(5.0) < ros::Time::now()) {
                last_error = ros::Time::now();
            }
            tf_error.clear();
        }

        if (has_static_layer_) {
            layered_costmap_->SetStaticFilePath(config_file_static_);
            Layer *plugin_static_layer;
            plugin_static_layer = new StaticLayer;
            layered_costmap_->AddPlugin(plugin_static_layer);
            plugin_static_layer->Initialize(layered_costmap_, map_name + "/" + "static_layer", &tf_);
        }

        if (has_obstacle_layer_) {
            layered_costmap_->SetObstacleFilePath(config_file_obstacle_);
            Layer *plugin_obstacle_layer = new ObstacleLayer;
            layered_costmap_->AddPlugin(plugin_obstacle_layer);
            plugin_obstacle_layer->Initialize(layered_costmap_, map_name + "/" + "obstacle_layer", &tf_);
        }

        layered_costmap_->SetInflationFilePath(config_file_inflation_);
        Layer *plugin_inflation_layer = new InflationLayer;
        layered_costmap_->AddPlugin(plugin_inflation_layer);
        plugin_inflation_layer->Initialize(layered_costmap_, map_name + "/" + "inflation_layer", &tf_);

        SetUnpaddedRobotFootprint(footprint_points_);
        stop_updates_ = false;
        initialized_ = true;
        stopped_ = false;
        robot_stopped_ = false;
        map_update_thread_shutdown_ = false;
        timer_ = private_nh.createTimer(ros::Duration(0.1), &CostmapInterface::DetectMovement, this);

        if (cost_translation_table_ == NULL) {
            cost_translation_table_ = new char[256];
            // special values:
            cost_translation_table_[0] = 0;  // NO obstacle
            cost_translation_table_[253] = 99;  // INSCRIBED obstacle
            cost_translation_table_[254] = 100;  // LETHAL obstacle
            cost_translation_table_[255] = -1;  // UNKNOWN

            // regular cost values scale the range 1 to 252 (inclusive) to fit
            // into 1 to 98 (inclusive).
            for (int i = 1; i < 253; i++) {
                cost_translation_table_[i] = char(1 + (97 * (i - 1)) / 251);
            }
        }

        costmap_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>(name_ + "/costmap", 10);
        if (get_footprint_)
            footprint_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>(name_ + "/footprint", 1);
        map_update_thread_ = new std::thread(std::bind(&CostmapInterface::MapUpdateLoop, this, map_update_frequency_));
        if (is_rolling_window_) {
            layered_costmap_->ResizeMap((unsigned int) map_width_ / map_resolution_,
                                        (unsigned int) map_height_ / map_resolution_,
                                        map_resolution_,
                                        map_origin_x_,
                                        map_origin_y_);
        }
    }

    void CostmapInterface::LoadParameter() {

        ParaCollection ParaCollectionConfig;
        or_common::ReadProtoFromTextFile(config_file_.c_str(), &ParaCollectionConfig);


        config_file_ = ros::package::getPath("or_costmap") +
                       ParaCollectionConfig.para_costmap_interface().inflation_file_path();

        map_update_frequency_ = ParaCollectionConfig.para_costmap_interface().map_update_frequency();
        global_frame_ = ParaCollectionConfig.para_costmap_interface().global_frame();
        robot_base_frame_ = ParaCollectionConfig.para_costmap_interface().robot_base_frame();
        footprint_padding_ = ParaCollectionConfig.para_costmap_interface().footprint_padding();
        transform_tolerance_ = ParaCollectionConfig.para_costmap_interface().transform_tolerance();
        is_rolling_window_ = ParaCollectionConfig.para_costmap_interface().is_rolling_window();
        is_track_unknown_ = ParaCollectionConfig.para_costmap_interface().is_tracking_unknown();
        has_obstacle_layer_ = ParaCollectionConfig.para_costmap_interface().has_obstacle_layer();
        has_static_layer_ = ParaCollectionConfig.para_costmap_interface().has_static_layer();
        map_width_ = ParaCollectionConfig.para_costmap_interface().map_width();
        map_height_ = ParaCollectionConfig.para_costmap_interface().map_height();
        map_origin_x_ = ParaCollectionConfig.para_costmap_interface().map_origin_x();
        map_origin_y_ = ParaCollectionConfig.para_costmap_interface().map_origin_y();
        map_resolution_ = ParaCollectionConfig.para_costmap_interface().map_resolution();


        config_file_inflation_ = ros::package::getPath("or_costmap") +
                                 ParaCollectionConfig.para_costmap_interface().inflation_file_path();
        if (has_obstacle_layer_)
            config_file_obstacle_ = ros::package::getPath("or_costmap") +
                                    ParaCollectionConfig.para_costmap_interface().obstacle_file_path();
        if (has_obstacle_layer_)
            config_file_static_ = ros::package::getPath("or_costmap") +
                                  ParaCollectionConfig.para_costmap_interface().static_file_path();


        geometry_msgs::Point point;
        for (auto i = 0; i < ParaCollectionConfig.footprint().point().size(); i++) {
            point.x = ParaCollectionConfig.footprint().point(i).x();
            point.y = ParaCollectionConfig.footprint().point(i).y();
            point.z = 0.0;
            footprint_points_.push_back(point);
        }

        get_footprint_ = ParaCollectionConfig.para_costmap_interface().get_footprint();
    }

    CostmapInterface::~CostmapInterface() {
        timer_.stop();
        map_update_thread_shutdown_ = true;
        if (map_update_thread_ != NULL) {
            map_update_thread_->join();
            delete map_update_thread_;
        }
        delete layered_costmap_;
        delete cost_translation_table_;
    }

    //设置机器人多边形
    void CostmapInterface::SetUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon &footprint) {
        SetUnpaddedRobotFootprint(ToPointVector(footprint));
    }

    //设置机器人多边形
    void CostmapInterface::SetUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point> &points) {
        unpadded_footprint_ = points;
        padded_footprint_ = points;
        PadFootprint(padded_footprint_, footprint_padding_);
        layered_costmap_->SetFootprint(padded_footprint_);
    }

    //检测机器人是否移动
    void CostmapInterface::DetectMovement(const ros::TimerEvent &event) {
        tf::Stamped<tf::Pose> new_pose;
        if (!GetRobotPose(new_pose)) {
            robot_stopped_ = false;
        } else if (fabs((old_pose_.getOrigin() - new_pose.getOrigin()).length()) < 1e-3 \
 && fabs(old_pose_.getRotation().angle(new_pose.getRotation())) < 1e-3) {
            old_pose_ = new_pose;
            robot_stopped_ = true;
        } else {
            old_pose_ = new_pose;
            robot_stopped_ = false;
        }
    }


    //求机器人坐标（tf格式带时间戳）
    bool CostmapInterface::GetRobotPose(tf::Stamped<tf::Pose> &global_pose) const {
        global_pose.setIdentity();
        tf::Stamped<tf::Pose> robot_pose;
        robot_pose.setIdentity();
        robot_pose.frame_id_ = robot_base_frame_;
        robot_pose.stamp_ = ros::Time();
        ros::Time current_time = ros::Time::now();
        try {
            tf_.transformPose(global_frame_, robot_pose, global_pose);
        }
        catch (tf::LookupException &ex) {
            ROS_ERROR("No Transform Error looking up robot pose: %s", ex.what());
            return false;
        }
        catch (tf::ConnectivityException &ex) {
            ROS_ERROR("Connectivity Error looking up robot pose: %s", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException &ex) {
            ROS_ERROR("Extrapolation Error looking up robot pose: %s", ex.what());
            return false;
        }
        // check global_pose timeout
        //if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_) {
        //  LOG_WARNING << "Interface transform timeout. Current time: " << current_time.toSec() << ", global_pose stamp: "
        //              << global_pose.stamp_.toSec() << ", tolerance: " << transform_tolerance_;
        //  return false;
        //}
        return true;
    }

    //求机器人坐标（tf格式->msg格式）
    bool CostmapInterface::GetRobotPose(geometry_msgs::PoseStamped &global_pose) const {
        tf::Stamped<tf::Pose> tf_global_pose;
        if (GetRobotPose(tf_global_pose)) {
            tf::poseStampedTFToMsg(tf_global_pose, global_pose);
            return true;
        } else {
            return false;
        }
    }

    //求机器人坐标（tf格式->Eigen格式）
    bool CostmapInterface::GetRobotPose(RobotPose &pose) {
        tf::Stamped<tf::Pose> global_pose;
        if (GetRobotPose(global_pose)) {
            pose.time = global_pose.stamp_;
            pose.frame_id = global_pose.frame_id_;
            pose.position
                    << global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), global_pose.getOrigin().getZ();
            auto mat = global_pose.getBasis();
            pose.rotation << mat.getRow(0).getX(), mat.getRow(0).getY(), mat.getRow(0).getZ(), \
        mat.getRow(1).getX(), mat.getRow(1).getY(), mat.getRow(1).getZ(), \
        mat.getRow(2).getX(), mat.getRow(2).getY(), mat.getRow(2).getZ();
            return true;
        }
        return false;
    }


    //得到机器人多变形的坐标,msg格式
    void CostmapInterface::GetOrientedFootprint(std::vector<geometry_msgs::Point> &oriented_footprint) const {
        tf::Stamped<tf::Pose> global_pose;
        if (!GetRobotPose(global_pose)) {
            return;
        }
        double yaw = tf::getYaw(global_pose.getRotation());
        TransformFootprint(global_pose.getOrigin().x(), global_pose.getOrigin().y(), yaw,
                           padded_footprint_, oriented_footprint);
    }

    //得到机器人多变形的坐标，从msg格式转成Eigen格式
    void CostmapInterface::GetOrientedFootprint(std::vector<Eigen::Vector3f> &footprint) {
        std::vector<geometry_msgs::Point> oriented_footprint;
        GetOrientedFootprint(oriented_footprint);
        Eigen::Vector3f position;
        for (auto it: oriented_footprint) {
            position << it.x, it.y, it.z;
            footprint.push_back(position);
        }
    }

    //将msg格式的footprint转成Eigen格式的
    void CostmapInterface::GetFootprint(std::vector<Eigen::Vector3f> &footprint) {
        std::vector<geometry_msgs::Point> ros_footprint = GetRobotFootprint();
        Eigen::Vector3f point;
        for (auto it: ros_footprint) {
            point << it.x, it.y, it.z;
            footprint.push_back(point);
        }
    }

    //输入pose_msg，求其在global_frame_坐标系下的位姿
    geometry_msgs::PoseStamped CostmapInterface::Pose2GlobalFrame(const geometry_msgs::PoseStamped &pose_msg) {
        tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
        poseStampedMsgToTF(pose_msg, tf_pose);

        tf_pose.stamp_ = ros::Time();
        try {
            tf_.transformPose(global_frame_, tf_pose, global_tf_pose);
        }
        catch (tf::TransformException &ex) {
            return pose_msg;
        }
        geometry_msgs::PoseStamped global_pose_msg;
        tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);
        return global_pose_msg;
    }

    //？这个函数还搞不懂存在的必要，其会将障碍物层 除机器人中心的cell值改成255（未知）
    void CostmapInterface::ClearCostMap() {
        std::vector<Layer *> *plugins = layered_costmap_->GetPlugins();
        tf::Stamped<tf::Pose> pose;
        if (GetRobotPose(pose) == false) {
            return;
        }
        double pose_x = pose.getOrigin().x();
        double pose_y = pose.getOrigin().y();

        for (std::vector<or_costmap::Layer *>::iterator plugin_iter = plugins->begin();
             plugin_iter != plugins->end();
             ++plugin_iter) {
            or_costmap::Layer *plugin = *plugin_iter;
            if (plugin->GetName().find("obstacle") != std::string::npos) {
                CostmapLayer *costmap_layer_ptr = (CostmapLayer *) plugin;
                ClearLayer(costmap_layer_ptr, pose_x, pose_y);
            }
        }
    }

    void CostmapInterface::ClearLayer(CostmapLayer *costmap_layer_ptr, double pose_x, double pose_y) {
        std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_layer_ptr->GetMutex()));
        double reset_distance = 0.1;
        double start_point_x = pose_x - reset_distance / 2;
        double start_point_y = pose_y - reset_distance / 2;
        double end_point_x = start_point_x + reset_distance;
        double end_point_y = start_point_y + reset_distance;

        int start_x, start_y, end_x, end_y;
        costmap_layer_ptr->World2MapNoBoundary(start_point_x, start_point_y, start_x, start_y);
        costmap_layer_ptr->World2MapNoBoundary(end_point_x, end_point_y, end_x, end_y);

        unsigned char *grid = costmap_layer_ptr->GetCharMap();
        for (int x = 0; x < (int) costmap_layer_ptr->GetSizeXCell(); x++) {
            bool xrange = x > start_x && x < end_x;

            for (int y = 0; y < (int) costmap_layer_ptr->GetSizeYCell(); y++) {
                if (xrange && y > start_y && y < end_y)
                    continue;
                int index = costmap_layer_ptr->GetIndex(x, y);
                if (grid[index] != NO_INFORMATION) {
                    grid[index] = NO_INFORMATION;
                }
            }
        }

        double ox = costmap_layer_ptr->GetOriginX(), oy = costmap_layer_ptr->GetOriginY();
        double width = costmap_layer_ptr->GetSizeXWorld(), height = costmap_layer_ptr->GetSizeYWorld();
        costmap_layer_ptr->AddExtraBounds(ox, oy, ox + width, oy + height);
    }


    void CostmapInterface::MapUpdateLoop(double frequency) {
        if (frequency <= 0.0) {
            ROS_ERROR("Frequency must be positive in MapUpdateLoop.");
        }
        ros::NodeHandle nh;
        ros::Rate r(frequency);
        while (nh.ok() && !map_update_thread_shutdown_) {
            struct timeval start, end;
            gettimeofday(&start, NULL);
            UpdateMap();
            gettimeofday(&end, NULL);
            r.sleep();
            if (r.cycleTime() > ros::Duration(1 / frequency)) {
                ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", \
        frequency, r.cycleTime().toSec());
            }
            double x, y;
            Costmap2D *temp_costmap = layered_costmap_->GetCostMap();
            unsigned char *data = temp_costmap->GetCharMap();
            temp_costmap->Map2World(0, 0, x, y);
            grid_.header.frame_id = global_frame_;
            grid_.header.stamp = ros::Time::now();
            if (is_rolling_window_) {
                grid_.info.resolution = map_resolution_;
                grid_.info.width = map_width_ / map_resolution_;
                grid_.info.height = map_height_ / map_resolution_;
                grid_.info.origin.position.x = x - map_resolution_ * 0.5;
                grid_.info.origin.position.y = y - map_resolution_ * 0.5;
                grid_.info.origin.position.z = 0;
                grid_.info.origin.orientation.w = 1.0;
                grid_.data.resize(grid_.info.width * grid_.info.height);
            } else {
                auto resolution = temp_costmap->GetResolution();
                auto map_width = temp_costmap->GetSizeXCell();
                auto map_height = temp_costmap->GetSizeYCell();
                grid_.info.resolution = resolution;
                grid_.info.width = map_width;
                grid_.info.height = map_height;
                grid_.info.origin.position.x = temp_costmap->GetOriginX();
                grid_.info.origin.position.y = temp_costmap->GetOriginY();
                grid_.info.origin.position.z = 0;
                grid_.info.origin.orientation.w = 1.0;
                grid_.data.resize(map_width * map_height);
            }
            for (size_t i = 0; i < grid_.data.size(); i++) {
                grid_.data[i] = cost_translation_table_[data[i]];
            }
            costmap_pub_.publish(grid_);
        }
    }

    //更新LayeredCostmap
    void CostmapInterface::UpdateMap() {
        if (!stop_updates_) {
            tf::Stamped<tf::Pose> pose;
            if (GetRobotPose(pose)) {
                double x = pose.getOrigin().x(), y = pose.getOrigin().y(),
                        yaw = tf::getYaw(pose.getRotation());
                layered_costmap_->UpdateMap(x, y, yaw);

                if (get_footprint_ && footprint_pub_.getNumSubscribers() > 0) {
                    //求机器人多边形在global_frame_中坐标，并发布
                    geometry_msgs::PolygonStamped footprint;
                    footprint.header.frame_id = global_frame_;
                    footprint.header.stamp = ros::Time::now();
                    TransformFootprint(x, y, yaw, padded_footprint_, footprint);
                    footprint_pub_.publish(footprint);
                }

                initialized_ = true;
            }
        }
    }

    void CostmapInterface::Start() {
        auto *plugins = layered_costmap_->GetPlugins();
        if (stopped_) {
            for (auto plugin = plugins->begin(); plugin != plugins->end(); ++plugin) {
                (*plugin)->Activate();
            }
            stopped_ = false;
        }
        stop_updates_ = false;
        ros::Rate r(100.0);
        while (ros::ok() && !initialized_) {
            r.sleep();
        }
    }

    void CostmapInterface::Stop() {
        stop_updates_ = true;
        auto *plugins = layered_costmap_->GetPlugins();
        for (auto plugin = plugins->begin(); plugin != plugins->end(); ++plugin) {
            (*plugin)->Deactivate();
        }
        initialized_ = false;
        stopped_ = true;
    }

    void CostmapInterface::Pause() {
        stop_updates_ = true;
        initialized_ = false;
    }

    void CostmapInterface::Resume() {
        stop_updates_ = false;
        ros::Rate r(100.0);
        while (!initialized_) {
            r.sleep();
        }
    }
}