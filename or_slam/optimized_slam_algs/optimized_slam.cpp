#include "optimized_slam.h"

//求Tbase_LMi

namespace optimizedSlam {
    OptimizedSlam::OptimizedSlam(const Eigen::Vector3d &init_pose, ros::NodeHandle *nh) {

        nh->param<double>("odom_alpha1", odom_translation_weight_, 1e4);
        nh->param<double>("odom_alpha2", odom_rotation_weight_, 1e4);
        nh->param<double>("odom_alpha3", lm_translation_weight_, 1e5);
        nh->param<double>("odom_alpha4", lm_rotation_weight_, 0.0);
        nh->param<double>("update_min_d", update_min_d_, 0.2); //在执行滤波更新前平移运动的距离0.05(50mm)与0.03（1.7度）
        nh->param<double>("update_min_a", update_min_a_, 0.2);
        Eigen::Matrix2d cov_z;
        cov_z.setZero();
        nh->param<double>("lm_cov_x", cov_z(0, 0), 0.8);
        nh->param<double>("lm_cov_y", cov_z(1, 1), 0.8);
        cov_z_inv_ = cov_z.inverse();

        init_global_pose_ = transform::Rigid2d({init_pose.x(), init_pose.y()}, init_pose.z());
        last_ros_odom_pose_.setZero();
    }

    OptimizedSlam::~OptimizedSlam() {
    }

    void OptimizedSlam::SetSensorPose(const Eigen::Vector3d &sensor_pose) {
    }

    const std::map<int, Eigen::Vector3d> OptimizedSlam::GetLandmarks() {
        std::map<int, Eigen::Vector3d> output;
        for (auto &landmark_node : landmark_data_) {
            output.insert({landmark_node.first, EigenV3FromPose(landmark_node.second.global_landmark_pose)});
        }
        return output;
    };

    const Eigen::Vector3d OptimizedSlam::GetPose(const Eigen::Vector3d &odom_pose) {

        transform::Rigid2d pose = init_global_pose_ * transform::Rigid2d({odom_pose.x(), odom_pose.y()}, odom_pose.z());
        if (!node_data_.empty()) {
            int node_id = std::prev(node_data_.end())->first;
            pose = (node_data_[node_id].global_pose_2d) * (node_data_[node_id].odom_pose_2d.inverse() * pose);
        }

        return EigenV3FromPose(pose);
    }

    void OptimizedSlam::AddNodeData(const Eigen::Vector3d &ros_odom_pose, const std::vector<double> &XYs,
                                    const ros::Time &stamp) {
        Eigen::Vector3d delta;
        delta[0] = ros_odom_pose[0] - last_ros_odom_pose_[0];
        delta[1] = ros_odom_pose[1] - last_ros_odom_pose_[1];
        delta[2] = optimizedSlam::common::NormalizeAngleDifference<double>(
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

        //最大似然法求lmi
        if (XYs.empty() || XYs.size() % 2 == 1)
            return;
        for (int i = 0; i < XYs.size(); i = i + 2) {
            Eigen::Vector2d xy{XYs[i], XYs[i + 1]};
            CalculateLikelihood((--node_data_.end()), xy);
        }

        Solve();
    }

    void OptimizedSlam::CalculateLikelihood(std::map<int, NodeSpec2D>::iterator &it, Eigen::Vector2d &xy) {
//        Eigen::Vector2d z_hat = it->second.global_pose_2d * xy;//预测的树干坐标
//        double min_error = std::numeric_limits<double>::max();
//        int min_lm_id;//最佳的树干编号
//        for (auto &landmark_node : landmark_data_) {
//            Eigen::Vector2d z = landmark_node.second.global_landmark_pose.translation();//树干“真实“的坐标
//            double error = (z - z_hat).transpose() * cov_z_inv_ * (z - z_hat);
//            if (error < min_error) {
//                min_error = error;
//                min_lm_id = landmark_node.first;
//            }
//        }

        Eigen::Vector2d z_hat;
        z_hat << atan2(xy.y(), xy.x()), xy.norm();

        double min_error = std::numeric_limits<double>::max();
        int min_lm_id;//最佳的树干编号
        transform::Rigid2d node_pose = it->second.global_pose_2d;
        for (auto &landmark_node : landmark_data_) {
            Eigen::Vector2d Tnode_lmi =
                    node_pose.inverse() * landmark_node.second.global_landmark_pose.translation();
            Eigen::Vector2d z;
            z << atan2(Tnode_lmi.y(), Tnode_lmi.x()), Tnode_lmi.norm();
            double error = (z - z_hat).transpose() * cov_z_inv_ * (z - z_hat);
            if (error < min_error) {
                min_error = error;
                min_lm_id = landmark_node.first;
            }
        }

        std::cout << "xy: " << xy(0) << " " << xy(1) << ";" << std::endl;
        std::cout << "z_hat: " << z_hat(0) << " " << z_hat(1) << ";" << std::endl;

        if (min_error < 5.99) {
            landmark_data_[min_lm_id].landmark_observations.push_back(
                    optimizedSlam::optimization::LandmarkNode::LandmarkObservation{
                            it->first, transform::Rigid2d(xy, it->second.global_pose_2d.rotation().inverse())
                    });

            //            std::cout << "Node " << it->second.global_pose_2d << " LM " << (min_lm_id) << ": "
//                      << landmark_data_[min_lm_id].global_landmark_pose << std::endl;
        } else {
            int last_lm_id = -1;
            if (!landmark_data_.empty()) {
                last_lm_id = std::prev(landmark_data_.end())->first;
            }
            transform::Rigid2d Tglobal_lmi(it->second.global_pose_2d * xy, 0.0);

            landmark_data_[last_lm_id + 1].global_landmark_pose = Tglobal_lmi;

            landmark_data_[last_lm_id + 1].landmark_observations.push_back(
                    optimizedSlam::optimization::LandmarkNode::LandmarkObservation{
                            it->first, it->second.global_pose_2d.inverse() * Tglobal_lmi});

//            std::cout << "Node " << it->second.global_pose_2d << " LM " << (last_lm_id + 1) << ": "
//                      << landmark_data_[last_lm_id + 1].global_landmark_pose << std::endl;
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
        problem.SetParameterBlockConstant(C_nodes.at(0).data());

        // 添加node之间在odom中的约束
        auto node_it = node_data_.begin();
        auto prev_node_it = node_it;
        for (++node_it; node_it != node_data_.end(); ++node_it) {
            const int first_node_id = prev_node_it->first;
            const NodeSpec2D &first_node_data = prev_node_it->second;
            prev_node_it = node_it;
            const int second_node_id = node_it->first;
            const NodeSpec2D &second_node_data = node_it->second;

            const transform::Rigid2d relative_local_slam_pose = first_node_data.odom_pose_2d.inverse() *
                                                                second_node_data.odom_pose_2d;
            problem.AddResidualBlock(
                    optimization::CreateAutoDiffSpaCostFunction(
                            optimization::PoseConstraint{relative_local_slam_pose,
                                                         odom_translation_weight_, odom_rotation_weight_}),
                    nullptr /* loss function */, C_nodes.at(first_node_id).data(),
                    C_nodes.at(second_node_id).data());
        }


        std::map<int, std::array<double, 3>> C_landmarks;

        for (const auto &landmark_node : landmark_data_) {
            int landmark_id = landmark_node.first;//lm的id

            for (const auto &observation : landmark_node.second.landmark_observations) {

                if (!C_landmarks.count(landmark_id)) {
                    //如果这个lmi还没有加入优化变量，就先给他算个初值

                    const transform::Rigid2d starting_point =
                            node_data_[observation.node_id].global_pose_2d *
                            observation.landmark_to_tracking_transform;
                    C_landmarks.emplace(landmark_id, FromPose(starting_point));

                    problem.AddParameterBlock(C_landmarks.at(landmark_id).data(), 3);
//                    problem.SetParameterBlockConstant(C_landmarks.at(landmark_id).begin() + 2);
                }

                problem.AddResidualBlock(
                        optimization::CreateAutoDiffSpaCostFunction(
                                optimization::PoseConstraint{observation.landmark_to_tracking_transform,
                                                             lm_translation_weight_, lm_rotation_weight_}),
                        nullptr /* loss function */, C_nodes.at(observation.node_id).data(),
                        C_landmarks.at(landmark_id).data());
            }
        }


        ceres::Solver::Options options;     // 这里有很多配置项可以填
        options.use_nonmonotonic_steps = false;
        options.max_num_iterations = 50;
        options.num_threads = 7;

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
            landmark_data_[C_landmark.first].global_landmark_pose = ToPose(C_landmark.second);
        }
    }

// Converts a pose into the 3 optimization variable format used for Ceres:
// translation in x and y, followed by the rotation angle representing the
// orientation.
    std::array<double, 3> OptimizedSlam::FromPose(const transform::Rigid2d &pose) {
        return {{pose.translation().x(), pose.translation().y(),
                        pose.normalized_angle()}};
    }

    Eigen::Vector3d OptimizedSlam::EigenV3FromPose(const transform::Rigid2d &pose) {
        return {pose.translation().x(), pose.translation().y(),
                pose.normalized_angle()};
    }

// Converts a pose as represented for Ceres back to an transform::Rigid2d pose.
    transform::Rigid2d OptimizedSlam::ToPose(const std::array<double, 3> &values) {
        return transform::Rigid2d({values[0], values[1]}, values[2]);
    }

} // namespace optimizedSlam