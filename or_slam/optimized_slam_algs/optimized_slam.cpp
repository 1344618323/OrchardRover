#include "optimized_slam.h"

//求Tbase_LMi

namespace optimized_slam {
    OptimizedSlam::OptimizedSlam(const Eigen::Vector3d &init_pose, ros::NodeHandle *nh, const bool &pure_localization)
            : pure_localization_(pure_localization) {
        nh->param<double>("optimized_slam/odom_translation_weight", odom_translation_weight_, 1e4);
        nh->param<double>("optimized_slam/odom_rotation_weight", odom_rotation_weight_, 1e4);
        nh->param<double>("optimized_slam/lm_translation_weight", lm_translation_weight_, 1e5);
        nh->param<double>("optimized_slam/lm_rotation_weight", lm_rotation_weight_, 0.0);
        nh->param<double>("optimized_slam/update_min_d", update_min_d_, 0.2);
        nh->param<double>("optimized_slam/update_min_a", update_min_a_, 0.2);
        Eigen::Matrix2d cov_z;
        cov_z.setZero();
        nh->param<double>("optimized_slam/lm_cov_x", cov_z(0, 0), 0.8);
        nh->param<double>("optimized_slam/lm_cov_y", cov_z(1, 1), 0.8);
        nh->param<int>("optimized_slam/reserve_node_num", reserve_node_num_, 300);

        if (pure_localization_)
            reserve_node_num_ = 5;

        std::cout << "asf:" << reserve_node_num_ << " " << lm_translation_weight_ << std::endl;
        cov_z_inv_ = cov_z.inverse();

        init_global_pose_ = transform::Rigid2d({init_pose.x(), init_pose.y()}, init_pose.z());
        last_ros_odom_pose_.setZero();
    }

    OptimizedSlam::~OptimizedSlam() {
    }

    void OptimizedSlam::SetSensorPose(const Eigen::Vector3d &sensor_pose) {
    }

    void OptimizedSlam::SetConstantLandmarks(const std::map<int, Eigen::Vector2d> &landmarks) {
        for (auto &landmark_node : landmarks) {
            landmark_data_[landmark_node.first].global_landmark_xy = landmark_node.second;
            landmark_data_[landmark_node.first].constant = true;
        }
    }

    const std::map<int, Eigen::Vector2d> OptimizedSlam::GetLandmarks() {
        std::unique_lock<std::mutex> lm_lock(mutex_landmarks_);
        return latest_landmarks_;
    };

    const std::vector<ResidualForVisualize> OptimizedSlam::GetResidualForVisualize() {
        std::unique_lock<std::mutex> lm_lock(mutex_landmarks_);
        std::vector<ResidualForVisualize> out;
        std::swap(out, residuals_for_visualize_);
        return out;
    }


    const Eigen::Vector3d OptimizedSlam::GetPose(const Eigen::Vector3d &odom_pose) {
        std::unique_lock<std::mutex> node_lock(mutex_latest_node_);
        transform::Rigid2d pose = init_global_pose_ * transform::Rigid2d({odom_pose.x(), odom_pose.y()}, odom_pose.z());
        if (!node_data_.empty()) {
            pose = latest_node_.global_pose_2d * latest_node_.odom_pose_2d.inverse() * pose;
        }
        return EigenV3FromPose(pose);
    }

    void OptimizedSlam::AddNodeData(const Eigen::Vector3d &ros_odom_pose, const std::vector<double> &XYs,
                                    const ros::Time &stamp) {

        Eigen::Vector3d delta;
        delta[0] = ros_odom_pose[0] - last_ros_odom_pose_[0];
        delta[1] = ros_odom_pose[1] - last_ros_odom_pose_[1];
        delta[2] = optimized_slam::common::NormalizeAngleDifference<double>(
                ros_odom_pose[2] - last_ros_odom_pose_[2]);

        // See if we should update the filter
        bool odom_update = std::fabs(delta[0]) > update_min_d_ ||
                           std::fabs(delta[1]) > update_min_d_ ||
                           std::fabs(delta[2]) > update_min_a_;

        if ((!odom_update) && algs_init_)
            return;

        if (!algs_init_)
            algs_init_ = true;

        last_ros_odom_pose_ = ros_odom_pose;

        transform::Rigid2d odom_pose =
                init_global_pose_ * transform::Rigid2d({ros_odom_pose.x(), ros_odom_pose.y()}, ros_odom_pose.z());

//        std::cout << "!odom_pose" << odom_pose << std::endl;

        if (node_data_.empty()) {
            node_data_[0] = NodeSpec2D{stamp, odom_pose, odom_pose};
        } else {
            if (XYs.empty())
                return;//原则上是进入不了这个条件的
            int last_node_id = std::prev(node_data_.end())->first;
            node_data_[last_node_id + 1] = NodeSpec2D{stamp, odom_pose,
                                                      (node_data_[last_node_id].global_pose_2d) *
                                                      (node_data_[last_node_id].odom_pose_2d.inverse() *
                                                       odom_pose)};
        }

        //最大似然法求lmi；卡方检测判断是否新增lmi
        if (XYs.empty() || XYs.size() % 2 == 1)
            return;
        for (int i = 0; i < XYs.size(); i = i + 2) {
            Eigen::Vector2d xy{XYs[i], XYs[i + 1]};
            CalculateLikelihood((--node_data_.end()), xy);
        }

        Solve();

        LandmarksAndNodeCulling();

        {
            std::unique_lock<std::mutex> node_lock(mutex_latest_node_);
            int node_id = std::prev(node_data_.end())->first;
            latest_node_ = node_data_[node_id];

            std::unique_lock<std::mutex> lm_lock(mutex_landmarks_);
            latest_landmarks_.clear();
            for (auto &landmark_node : landmark_data_) {
                latest_landmarks_.insert({landmark_node.first, landmark_node.second.global_landmark_xy});

                for (auto &obs:landmark_node.second.landmark_observations) {
                    auto &node_pose = node_data_[obs.node_id].global_pose_2d;
                    residuals_for_visualize_.push_back(
                            {node_pose.translation(),
                             node_pose * obs.landmark_to_tracking_transform,
                             landmark_node.second.global_landmark_xy});
                }
            }
        }
    }

    void OptimizedSlam::CalculateLikelihood(std::map<int, NodeSpec2D>::iterator &it, Eigen::Vector2d &xy) {
//        Eigen::Vector2d z(atan2(xy.y(), xy.x()), xy.norm());
//        Eigen::Vector2d z_hat_min;
//
//        double min_error = std::numeric_limits<double>::max();
//        int min_lm_id;//最佳的树干编号
//        transform::Rigid2d node_pose = it->second.global_pose_2d;//当前观测node的位姿
//
//        for (auto &landmark_node : landmark_data_) {
//            Eigen::Vector2d lmi_in_node_frame =
//                    node_pose.inverse() * landmark_node.second.global_landmark_xy;
//
//            Eigen::Vector2d z_hat(atan2(lmi_in_node_frame.y(), lmi_in_node_frame.x()), lmi_in_node_frame.norm());
//
//            double error = (z - z_hat).transpose() * cov_z_inv_ * (z - z_hat);
//            if (error < min_error) {
//                min_error = error;
//                min_lm_id = landmark_node.first;
//                z_hat_min = z_hat;
//            }
//        }

        Eigen::Vector2d z = xy;
        Eigen::Vector2d z_hat_min;
        double min_error = std::numeric_limits<double>::max();
        transform::Rigid2d &node_pose = it->second.global_pose_2d;//当前观测node的位姿
        int min_lm_id;//最佳的树干编号
        for (auto &landmark_node : landmark_data_) {

            Eigen::Vector2d z_hat =
                    node_pose.inverse() * landmark_node.second.global_landmark_xy;
            double error = (z - z_hat).transpose() * cov_z_inv_ * (z - z_hat);
            if (error < min_error) {
                min_error = error;
                min_lm_id = landmark_node.first;
                z_hat_min = z_hat;
            }
        }

        std::cout << "z: " << z(0) << " " << z(1) << "; " << "z_hat_min: " << z_hat_min(0) << " " << z_hat_min(1)
                  << "; " << "chi-square: " << min_error << std::endl;

        int obs_lm_id = -1;
        if (min_error < 5.99) {
            obs_lm_id = min_lm_id;
        } else {
            if (!pure_localization_) {
                int last_lm_id = -1;
                if (!landmark_data_.empty()) {
                    last_lm_id = std::prev(landmark_data_.end())->first;
                }
                Eigen::Vector2d lmi_in_global_frame = it->second.global_pose_2d * xy;
                obs_lm_id = last_lm_id + 1;
                landmark_data_[obs_lm_id].global_landmark_xy = lmi_in_global_frame;
            }
        }

        if (obs_lm_id != -1) {
            landmark_data_[obs_lm_id].landmark_observations.push_back(
                    optimized_slam::optimization::LandmarkNode::LandmarkObservation{
                            it->first, xy});
            //            std::cout << "Node " << it->second.global_pose_2d << " LM " << (obs_lm_id) << ": "
            //              << landmark_data_[obs_lm_id].global_landmark_xy << std::endl;
        }
    }

    void OptimizedSlam::Solve() {
        if (node_data_.empty()) {
            return;
        }

        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);

        //往problem里插入node，第一个node设为固定值
        std::map<int, std::array<double, 3>> C_nodes;
        for (const auto &node_id_data : node_data_) {
            C_nodes[node_id_data.first] = FromPose(node_id_data.second.global_pose_2d);
            problem.AddParameterBlock(C_nodes.at(node_id_data.first).data(), 3);
        }
        problem.SetParameterBlockConstant(C_nodes.begin()->second.data());

        // 添加node之间在odom中的约束
        auto node_it = node_data_.begin();
        auto prev_node_it = node_it;
        for (++node_it; node_it != node_data_.end(); ++node_it) {
            const int first_node_id = prev_node_it->first;
            const NodeSpec2D &first_node_data = prev_node_it->second;
            prev_node_it = node_it;
            const int second_node_id = node_it->first;
            const NodeSpec2D &second_node_data = node_it->second;

            const transform::Rigid2d relative_odom_pose = first_node_data.odom_pose_2d.inverse() *
                                                          second_node_data.odom_pose_2d;
            problem.AddResidualBlock(
                    optimization::CreateAutoDiffSpaCostFunction(
                            optimization::PoseConstraint{relative_odom_pose,
                                                         odom_translation_weight_, odom_rotation_weight_}),
                    nullptr /* loss function */, C_nodes.at(first_node_id).data(),
                    C_nodes.at(second_node_id).data());
        }


        std::map<int, std::array<double, 2>> C_landmarks;

        int land_obs = 0;

        for (const auto &landmark_node : landmark_data_) {
            int landmark_id = landmark_node.first;//lm的id

            for (const auto &observation : landmark_node.second.landmark_observations) {

                if (!C_landmarks.count(landmark_id)) {
                    C_landmarks[landmark_id] = FromXY(landmark_node.second.global_landmark_xy);
                    problem.AddParameterBlock(C_landmarks.at(landmark_id).data(), 2);

                    if (landmark_node.second.constant) {
                        problem.SetParameterBlockConstant(C_landmarks.at(landmark_id).data());
                    }
                }

                problem.AddResidualBlock(
                        optimization::CreateAutoDiffLmCostFunction(
                                optimization::PoselmConstraint{observation.landmark_to_tracking_transform,
                                                               lm_translation_weight_}),
                        nullptr /* loss function */, C_nodes.at(observation.node_id).data(),
                        C_landmarks.at(landmark_id).data());

                land_obs++;
            }
        }

        std::cout << "NumResidual:" << problem.NumResidualBlocks() << " NumParameter" << problem.NumParameterBlocks()
                  << "; land_obs:" << land_obs << " nodes:" << node_data_.size() << " lm:" << landmark_data_.size()
                  << std::endl;

        ceres::Solver::Options options;
//        options.use_nonmonotonic_steps = true;
        options.gradient_tolerance = 1e-16;
        options.function_tolerance = 1e-16;
        options.max_num_iterations = 50;
//        options.num_threads = 7;

        ceres::Solver::Summary summary;                // 优化信息
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        ceres::Solve(options, &problem, &summary);       // 开始优化
        std::cout << summary.BriefReport() << std::endl;
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(
                t2 - t1);
        std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;

        // Store the result.
        for (const auto &C_node_id_data : C_nodes) {
            node_data_.at(C_node_id_data.first).global_pose_2d =
                    ToPose(C_node_id_data.second);
        }
        for (const auto &C_landmark : C_landmarks) {
            landmark_data_[C_landmark.first].global_landmark_xy = ToXY(C_landmark.second);
        }
    }

    void OptimizedSlam::LandmarksAndNodeCulling() {

        //先删除没用的node
        int bound_node_id = TrimNodeData();
        std::cout << "bound_node_id: " << bound_node_id << std::endl;

        transform::Rigid2d &node_pose = std::prev(node_data_.end())->second.global_pose_2d;//当前观测node的位姿
        int node_id = std::prev(node_data_.end())->first;

        std::vector<int> culling_lms_id;
        for (auto &landmark_node : landmark_data_) {

            /***将lm对象中 被没用的node观测 的数据 删除掉***/
            auto &obs = landmark_node.second.landmark_observations;
            auto obs_it = obs.begin();
            auto obs_end_it = obs.end();
            while (obs_it != obs_end_it && obs_it->node_id < bound_node_id) {
                obs.erase(obs_it);
                obs_it = obs.begin();
                obs_end_it = obs.end();
            }

            /***若lm被 这次新增加的node 观察到，一定不会删除这个lm；固定的lm也不会删除***/
            bool current_obs = false;

            if (node_id == std::prev(obs.end())->node_id) {
                current_obs = true;
                landmark_node.second.visible++;
                landmark_node.second.found++;
            }

            if (current_obs || landmark_node.second.constant)
                continue;

            /***优化后应该被看到，当时反而没看到: 按照模拟器的设置，在4m [-60,60]以内，应该能看到，我们收紧一点***/
            Eigen::Vector2d lmi_in_node_frame =
                    node_pose.inverse() * landmark_node.second.global_landmark_xy;

            Eigen::Vector2d z_hat(atan2(lmi_in_node_frame.y(), lmi_in_node_frame.x()), lmi_in_node_frame.norm());
            if ((z_hat[0] < M_PI_4 && z_hat[0] > -M_PI_4 && z_hat[1] < 3.5)) {
                landmark_node.second.visible++;
                if (double(landmark_node.second.found) / (landmark_node.second.visible) <= 0.25) {
                    culling_lms_id.push_back(landmark_node.first);
                }
            }
        }

        for (auto &id:culling_lms_id) {
            landmark_data_.erase(id);
        }
    }

    //返回最开始node的id
    int OptimizedSlam::TrimNodeData() {
        while (node_data_.size() > reserve_node_num_) {
            node_data_.erase(node_data_.begin());
        }
        return node_data_.begin()->first;
    }

    std::array<double, 3> OptimizedSlam::FromPose(const transform::Rigid2d &pose) {
        return {{pose.translation().x(), pose.translation().y(),
                        pose.normalized_angle()}};
    }

    std::array<double, 2> OptimizedSlam::FromXY(const Eigen::Vector2d &xy) {
        return {{xy.x(), xy.y()}};
    }

    Eigen::Vector3d OptimizedSlam::EigenV3FromPose(const transform::Rigid2d &pose) {
        return {pose.translation().x(), pose.translation().y(),
                pose.normalized_angle()};
    }

    transform::Rigid2d OptimizedSlam::ToPose(const std::array<double, 3> &values) {
        return transform::Rigid2d({values[0], values[1]}, values[2]);
    }

    Eigen::Vector2d OptimizedSlam::ToXY(const std::array<double, 2> &values) {
        return Eigen::Vector2d(values[0], values[1]);
    }

} // namespace optimized_slam