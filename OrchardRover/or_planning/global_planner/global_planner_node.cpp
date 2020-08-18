#include "global_planner_node.h"

namespace or_global_planner {
    /*
     * PlanThread是规划线程：（用于规划，并对node_state_赋值）
     *   while(node_state_==running):
     *      调用规划算法；发布路径topic（如果规划没失败，且goal没出界）
     *      若规划连续3次失败 or goal出界，则node_state_=failure
     *      若机器人已到目标位置时，则node_state_=success
     *      线程靠plan_condition_的wait_for延时，默认是333ms一次循环
     *
     * GoalCallback是action执行线程(开启、终止、唤醒PlanThread)(另外，其只会在终止PlanThread线程时，才会修改node_state_)
     *   如检测到新goal
     *      若此时node_state_为IDLE，说明PlanThread是空的，则新建PlanThread（刚进入PlanThread时，默认wait_for延时0微秒）
     *      若此时node_state_为running，依靠plan_condition_唤醒PlanThread(也就是说若PlanThread在靠wait_for延时，直接就跳出延时)
     *   while(ros::ok()):
     *      若PlanThread中的规划算法有异常 或 规划算法发布了新路径，则发布反馈
     *      若node_state_为success or failure
     *              终止PlanThread线程运行（将node_state_修改为IDLE）
     */


    using or_common::ErrorCode;
    using or_common::ErrorInfo;
    using or_common::NodeState;

    GlobalPlannerNode::GlobalPlannerNode() :
            new_path_(false), pause_(false), node_state_(NodeState::IDLE), error_info_(ErrorCode::OK),
            as_(nh_, "global_planner_node_action", boost::bind(&GlobalPlannerNode::GoalCallback, this, _1), false) {

        if (Init().IsOK()) {
            ROS_INFO("Global planner initialization completed.");
            as_.start();
        } else {
            ROS_ERROR("Initialization failed.");
            SetNodeState(NodeState::FAILURE);
        }
    }

    GlobalPlannerNode::~GlobalPlannerNode() {
        StopPlanning();
    }

    ErrorInfo GlobalPlannerNode::Init() {

        // Load proto planning configuration parameters
        GlobalPlannerConfig global_planner_config;
        std::string full_path =
                ros::package::getPath("or_planning") + "/global_planner/config/global_planner_config.prototxt";
        if (!or_common::ReadProtoFromTextFile(full_path.c_str(),
                                              &global_planner_config)) {
            ROS_ERROR("Cannot load global planner protobuf configuration file.");
            return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                             "Cannot load global planner protobuf configuration file.");
        }


        selected_algorithm_ = global_planner_config.selected_algorithm();
        //microseconds是微秒，1e6μs=1s
        cycle_duration_ = std::chrono::microseconds((int) (1e6 / global_planner_config.frequency()));
        max_retries_ = global_planner_config.max_retries();
        goal_distance_tolerance_ = global_planner_config.goal_distance_tolerance();
        goal_angle_tolerance_ = global_planner_config.goal_angle_tolerance();

        // ROS path visualize
        ros::NodeHandle viz_nh("~");
        path_pub_ = viz_nh.advertise<nav_msgs::Path>("path", 10);

        // Create tf listener
        tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

        // Create global costmap
        std::string map_path = ros::package::getPath("or_costmap") +
                               "/config/costmap_parameter_config_for_global_plan.prototxt";
        costmap_ptr_ = std::make_shared<or_costmap::CostmapInterface>("global_costmap",
                                                                      *tf_ptr_,
                                                                      map_path.c_str());
        // Create the instance of the selected algorithm
        if (selected_algorithm_ == "a_star_planner")
            global_planner_ptr_ = std::make_unique<or_global_planner::AStarPlanner>(costmap_ptr_);
        if (global_planner_ptr_ == nullptr) {
            ROS_ERROR("global planner algorithm instance can't be loaded");
            return ErrorInfo(ErrorCode::GP_INITILIZATION_ERROR,
                             "global planner algorithm instance can't be loaded");
        }


        // Initialize path frame from global costmap
        path_.header.frame_id = costmap_ptr_->GetGlobalFrameID();
        return ErrorInfo(ErrorCode::OK);
    }

    void GlobalPlannerNode::GoalCallback(const or_msgs::GlobalPlannerGoal::ConstPtr &msg) {
        ROS_INFO("Received a Goal from client!");
        //Update current error and info
        ErrorInfo error_info = GetErrorInfo();
        NodeState node_state = GetNodeState();

        //Notify the condition variable to stop lock waiting the fixed duration
        //触发plan线程
        {
            std::unique_lock<std::mutex> plan_lock(plan_mutex_);
            SetGoal(msg->goal);
            plan_condition_.notify_one();
        }

        if (node_state == NodeState::IDLE) {
            StartPlanning();
        }

        while (ros::ok()) {
            // Preempted and Canceled
            if (as_.isPreemptRequested()) {
                if (as_.isNewGoalAvailable()) {
                    as_.setPreempted();
                    break;
                } else {
                    as_.setPreempted();
                    StopPlanning();
                    break;
                }
            }

            // Update the current state and error info
            node_state = GetNodeState();
            error_info = GetErrorInfo();
            if (node_state == NodeState::RUNNING || node_state == NodeState::SUCCESS ||
                node_state == NodeState::FAILURE) {
                or_msgs::GlobalPlannerFeedback feedback;
                or_msgs::GlobalPlannerResult result;
                // If error occurs or planner produce new path, publish the feedback
                if (!error_info.IsOK() || new_path_) {
                    if (!error_info.IsOK()) {
                        //如果有异常状态，就反馈异常类型、异常字符串
                        feedback.error_code = error_info.error_code();
                        feedback.error_msg = error_info.error_msg();
                        SetErrorInfo(ErrorInfo::OK());
                    }
                    if (new_path_) {
                        //若是新规划了路径，就反馈路径
                        feedback.path = path_;
                        new_path_ = false;
                    }
                    as_.publishFeedback(feedback);
                }

                // After get the result, deal with actionlib server and jump out of the loop
                if (node_state == NodeState::SUCCESS) {
                    result.error_code = error_info.error_code();
                    as_.setSucceeded(result, error_info.error_msg());
                    StopPlanning();
                    break;
                } else if (node_state == NodeState::FAILURE) {
                    result.error_code = error_info.error_code();
                    as_.setAborted(result, error_info.error_msg());
                    StopPlanning();
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void GlobalPlannerNode::PlanThread() {
        ROS_INFO("Plan thread start!");
        geometry_msgs::PoseStamped current_start;
        geometry_msgs::PoseStamped current_goal;
        std::vector<geometry_msgs::PoseStamped> current_path;
        std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
        ErrorInfo error_info;
        int retries = 0;
        while (GetNodeState() == NodeState::RUNNING) {
            // 当 std::condition_variable 对象的某个 wait 函数被调用的时候，
            // 它使用 std::unique_lock(通过 std::mutex) 来锁住当前线程。
            // 调用wait(),当前线程会一直被阻塞，直到另外一个线程在相同的 std::condition_variable 对象上调用了 notification 函数来唤醒当前线程
            // 用的是wait_for(),会等待一段时间，如果被唤醒，立刻执行下去；否则等到延时结束，再执行下去
            // 这里的plan_condition_就是 std::condition_variable对象，是通过plan_mutex_来锁住线程

            ROS_INFO("Wait to plan!");
            std::unique_lock<std::mutex> plan_lock(plan_mutex_);
            plan_condition_.wait_for(plan_lock, sleep_time);
            ROS_INFO("Go on planning!");

            std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

            {
                std::unique_lock<or_costmap::Costmap2D::mutex_t> lock(*(costmap_ptr_->GetCostMap()->GetMutex()));
                bool error_set = false;
                //Get the robot current pose
                while (!costmap_ptr_->GetRobotPose(current_start)) {
                    if (!error_set) {
                        ROS_ERROR("Get Robot Pose Error.");
                        SetErrorInfo(ErrorInfo(ErrorCode::GP_GET_POSE_ERROR, "Get Robot Pose Error."));
                        error_set = true;
                    }
                    std::this_thread::sleep_for(std::chrono::microseconds(1));
                }

                //Get the robot current goal and transform to the global frame
                current_goal = GetGoal();

                if (current_goal.header.frame_id != costmap_ptr_->GetGlobalFrameID()) {
                    current_goal = costmap_ptr_->Pose2GlobalFrame(current_goal);
                    SetGoal(current_goal);
                }

                //Plan
                error_info = global_planner_ptr_->Plan(current_start, current_goal, current_path);

            }

            if (error_info.IsOK()) {
                //When planner succeed, reset the retry times
                retries = 0;
                PathVisualization(current_path);

                //Set the goal to avoid the same goal from getting transformed every time
                current_goal = current_path.back();
                SetGoal(current_goal);

                //Decide whether robot reaches the goal according to tolerance
                if (GetDistance(current_start, current_goal) < goal_distance_tolerance_
                    && GetAngle(current_start, current_goal) < goal_angle_tolerance_
                        ) {
                    //机器人到达目标点，action成功
                    SetNodeState(NodeState::SUCCESS);
                }
            } else if (max_retries_ > 0 && retries > max_retries_) {
                //When plan failed to max retries, return failure
                //多次规划后失败了，action失败
                ROS_ERROR("Can not get plan with max retries( %d )", max_retries_);
                error_info = ErrorInfo(ErrorCode::GP_MAX_RETRIES_FAILURE, "Over max retries.");
                SetNodeState(NodeState::FAILURE);
                retries = 0;
            } else if (error_info == ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR)) {
                //When goal is not reachable, return failure immediately
                //规划目标点超出了界限，action失败
                ROS_ERROR("Current goal is not valid!");
                SetNodeState(NodeState::FAILURE);
                retries = 0;
            } else {
                //Increase retries
                retries++;
                ROS_ERROR("Can not get plan for once. %s", error_info.error_msg().c_str());
            }
            // Set and update the error info
            SetErrorInfo(error_info);

            // Deal with the duration to wait
            std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
            std::chrono::microseconds execution_duration =
                    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            sleep_time = cycle_duration_ - execution_duration;

            // Report warning while planning timeout
            if (sleep_time <= std::chrono::microseconds(0)) {
                ROS_ERROR("The time planning once is %ld beyond the expected time %ld",
                          execution_duration.count(),
                          cycle_duration_.count());
                sleep_time = std::chrono::microseconds(0);
                SetErrorInfo(ErrorInfo(ErrorCode::GP_TIME_OUT_ERROR, "Planning once time out."));
            }
        }
        ROS_INFO("Plan thread terminated!");
    }


    NodeState GlobalPlannerNode::GetNodeState() {
        std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
        return node_state_;
    }

    void GlobalPlannerNode::SetNodeState(NodeState node_state) {
        std::lock_guard<std::mutex> node_state_lock(node_state_mtx_);
        node_state_ = node_state;
    }

    ErrorInfo GlobalPlannerNode::GetErrorInfo() {
        std::lock_guard<std::mutex> error_info_lock(error_info_mtx_);
        return error_info_;
    }

    void GlobalPlannerNode::SetErrorInfo(ErrorInfo error_info) {
        std::lock_guard<std::mutex> node_state_lock(error_info_mtx_);
        error_info_ = error_info;
    }

    geometry_msgs::PoseStamped GlobalPlannerNode::GetGoal() {
        std::lock_guard<std::mutex> goal_lock(goal_mtx_);
        return goal_;
    }

    void GlobalPlannerNode::SetGoal(geometry_msgs::PoseStamped goal) {
        std::lock_guard<std::mutex> goal_lock(goal_mtx_);
        goal_ = goal;
    }

    //开启规划线程
    void GlobalPlannerNode::StartPlanning() {
        if (plan_thread_.joinable()) {
            plan_thread_.join();
        }
        SetNodeState(or_common::NodeState::RUNNING);
        plan_thread_ = std::thread(&GlobalPlannerNode::PlanThread, this);
    }

    //停止规划线程
    void GlobalPlannerNode::StopPlanning() {
        SetNodeState(or_common::IDLE);
        if (plan_thread_.joinable()) {
            plan_thread_.join();
        }
    }

    void GlobalPlannerNode::PathVisualization(const std::vector<geometry_msgs::PoseStamped> &path) {
        path_.poses = path;
        path_pub_.publish(path_);
        new_path_ = true;
    }

    double GlobalPlannerNode::GetDistance(const geometry_msgs::PoseStamped &pose1,
                                          const geometry_msgs::PoseStamped &pose2) {
        const geometry_msgs::Point point1 = pose1.pose.position;
        const geometry_msgs::Point point2 = pose2.pose.position;
        const double dx = point1.x - point2.x;
        const double dy = point1.y - point2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double GlobalPlannerNode::GetAngle(const geometry_msgs::PoseStamped &pose1,
                                       const geometry_msgs::PoseStamped &pose2) {
        const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
        const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
        tf::Quaternion rot1, rot2;
        tf::quaternionMsgToTF(quaternion1, rot1);
        tf::quaternionMsgToTF(quaternion2, rot2);
        return rot1.angleShortestPath(rot2);
    }

} //namespace or_global_planner

int main(int argc, char **argv) {
    ros::init(argc, argv, "global_planner_node");
    or_global_planner::GlobalPlannerNode global_planner;
    ros::spin();
    return 0;
}
