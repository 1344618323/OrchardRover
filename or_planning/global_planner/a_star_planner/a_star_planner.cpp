#include "a_star_planner.h"

namespace or_global_planner {
/*
 * A* 流程
 * 优先队列q 根据 f(n)=g(n)+h(n)排序
 * while(!q.empty()):
 *   从q中pop出 node n，mark一下 n
 *   neighbor(n)=ms
 *   for m in ms：
 *     if m 没有被mark
 *       if g(m)>g(n)+Cnm    Cnm：从n到m的移动代价，在costmap中还包括m处的cost
 *         g(m)=g(n)+Cnm
 *       if m 不在队列中：     值得一提的是，即便m已经在队列中，若其g(m)在上一步缩小了，也没必要将m从q中去掉，再重新加入q中
 *         f(m)=g(m)+h(m)
 *         m加入q中
 */

    using or_common::ErrorCode;
    using or_common::ErrorInfo;

    AStarPlanner::AStarPlanner(CostmapPtr costmap_ptr) :
            GlobalPlannerBase::GlobalPlannerBase(costmap_ptr),
            gridmap_width_(costmap_ptr_->GetCostMap()->GetSizeXCell()),
            gridmap_height_(costmap_ptr_->GetCostMap()->GetSizeYCell()),
            cost_(costmap_ptr_->GetCostMap()->GetCharMap()) {

        AStarPlannerConfig a_star_planner_config;
        std::string full_path = ros::package::getPath("or_planning") + "/global_planner/a_star_planner/"
                                                                       "config/a_star_planner_config.prototxt";

        if (!or_common::ReadProtoFromTextFile(full_path.c_str(),
                                              &a_star_planner_config)) {
            ROS_ERROR("Cannot load a star planner protobuf configuration file.");
        }
        //  AStarPlanner param config
        heuristic_factor_ = a_star_planner_config.heuristic_factor();
        inaccessible_cost_ = a_star_planner_config.inaccessible_cost();
        goal_search_tolerance_ =
                a_star_planner_config.goal_search_tolerance() / costmap_ptr->GetCostMap()->GetResolution();
    }

    AStarPlanner::~AStarPlanner() {
        cost_ = nullptr;
    }

    ErrorInfo AStarPlanner::Plan(const geometry_msgs::PoseStamped &start,
                                 const geometry_msgs::PoseStamped &goal,
                                 std::vector<geometry_msgs::PoseStamped> &path) {

        unsigned int start_x, start_y, goal_x, goal_y, tmp_goal_x, tmp_goal_y;
        unsigned int valid_goal[2];
        unsigned int shortest_dist = std::numeric_limits<unsigned int>::max();
        bool goal_valid = false;

        if (!costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x,
                                                   start.pose.position.y,
                                                   start_x,
                                                   start_y)) {
            ROS_WARN("Failed to transform start pose from map frame to costmap frame");
            return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                             "Start pose can't be transformed to costmap frame.");
        }
        if (!costmap_ptr_->GetCostMap()->World2Map(goal.pose.position.x,
                                                   goal.pose.position.y,
                                                   goal_x,
                                                   goal_y)) {
            ROS_WARN("Failed to transform goal pose from map frame to costmap frame");
            return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                             "Goal pose can't be transformed to costmap frame.");
        }

        //(cxn)若设置的终点的cost在253以下就是可用的终点
        //否则在 {[gx-goal_search_tolerance_,gx+goal_search_tolerance_],y同理} 中搜索不是障碍物的，且cost最小的点 作为可用的终点
        if (costmap_ptr_->GetCostMap()->GetCost(goal_x, goal_y) < inaccessible_cost_) {
            valid_goal[0] = goal_x;
            valid_goal[1] = goal_y;
            goal_valid = true;
        } else {
            tmp_goal_x = goal_x;
            tmp_goal_y = goal_y - goal_search_tolerance_;

            while (tmp_goal_y <= goal_y + goal_search_tolerance_) {
                tmp_goal_x = goal_x - goal_search_tolerance_;
                while (tmp_goal_x <= goal_x + goal_search_tolerance_) {
                    unsigned char cost = costmap_ptr_->GetCostMap()->GetCost(tmp_goal_x, tmp_goal_y);
                    unsigned int dist = abs(goal_x - tmp_goal_x) + abs(goal_y - tmp_goal_y);
                    if (cost < inaccessible_cost_ && dist < shortest_dist) {
                        shortest_dist = dist;
                        valid_goal[0] = tmp_goal_x;
                        valid_goal[1] = tmp_goal_y;
                        goal_valid = true;
                    }
                    tmp_goal_x += 1;
                }
                tmp_goal_y += 1;
            }
        }
        ErrorInfo error_info;
        if (!goal_valid) {
            error_info = ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR);
            path.clear();
        } else {
            unsigned int start_index, goal_index;
            start_index = costmap_ptr_->GetCostMap()->GetIndex(start_x, start_y);
            goal_index = costmap_ptr_->GetCostMap()->GetIndex(valid_goal[0], valid_goal[1]);

            //(cxn)盲猜这么搞,是为了避免机器人在start里出不来
            costmap_ptr_->GetCostMap()->SetCost(start_x, start_y, or_costmap::FREE_SPACE);

            if (start_index == goal_index) {
                error_info = ErrorInfo::OK();
                path.clear();
                path.push_back(start);
                path.push_back(goal);
            } else {
                error_info = SearchPath(start_index, goal_index, path);
                if (error_info.IsOK()) {
                    path.back().pose.orientation = goal.pose.orientation;
                    path.back().pose.position.z = goal.pose.position.z;
                }
            }
        }
        return error_info;
    }

    ErrorInfo AStarPlanner::SearchPath(const int &start_index,
                                       const int &goal_index,
                                       std::vector<geometry_msgs::PoseStamped> &path) {

        g_score_.clear();
        f_score_.clear();
        parent_.clear();
        state_.clear();
        gridmap_width_ = costmap_ptr_->GetCostMap()->GetSizeXCell();
        gridmap_height_ = costmap_ptr_->GetCostMap()->GetSizeYCell();
        ROS_INFO("Search in a map %d", gridmap_width_ * gridmap_height_);
        cost_ = costmap_ptr_->GetCostMap()->GetCharMap();
        g_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
        f_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
        parent_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
        state_.resize(gridmap_height_ * gridmap_width_, SearchState::NOT_HANDLED);

        std::priority_queue<int, std::vector<int>, Compare> openlist;
        g_score_.at(start_index) = 0;
        openlist.push(start_index);

        std::vector<int> neighbors_index;
        int current_index, move_cost, h_score, count = 0;

        while (!openlist.empty()) {
            current_index = openlist.top();
            openlist.pop();
            state_.at(current_index) = SearchState::CLOSED;

            if (current_index == goal_index) {
                ROS_INFO("Search takes %d cycle counts", count);
                break;
            }

            //(cxn)8通道
            GetNineNeighbors(current_index, neighbors_index);

            for (auto neighbor_index : neighbors_index) {

                //若邻居cell出界，则跳过
                if (neighbor_index < 0 ||
                    neighbor_index >= gridmap_height_ * gridmap_width_) {
                    continue;
                }

                //(cxn)若cell值在253及以上，就是A*中的障碍物
                if (cost_[neighbor_index] >= inaccessible_cost_ ||
                    state_.at(neighbor_index) == SearchState::CLOSED) {
                    continue;
                }

                GetMoveCost(current_index, neighbor_index, move_cost);

                //(cxn)gCost还加上了 costmap 中的 cost，意味着邻居点距离障碍物越近，移动所需的代价越高
                if (g_score_.at(neighbor_index) > g_score_.at(current_index) + move_cost + cost_[neighbor_index]) {

                    //如neighbor_index已经在优先队列中，就算其g值发生变换
                    g_score_.at(neighbor_index) = g_score_.at(current_index) + move_cost + cost_[neighbor_index];
                    parent_.at(neighbor_index) = current_index;

                    if (state_.at(neighbor_index) == SearchState::NOT_HANDLED) {
                        GetManhattanDistance(neighbor_index, goal_index, h_score);
                        f_score_.at(neighbor_index) = g_score_.at(neighbor_index) + h_score;
                        openlist.push(neighbor_index);
                        state_.at(neighbor_index) = SearchState::OPEN;
                    }
                }
            }
            count++;
        }

        if (current_index != goal_index) {
            ROS_WARN("Global planner can't search the valid path!");
            return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR, "Valid global path not found.");
        }

        unsigned int iter_index = current_index, iter_x, iter_y;

        geometry_msgs::PoseStamped iter_pos;
        iter_pos.pose.orientation.w = 1;
        iter_pos.header.frame_id = "map";
        path.clear();
        costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
        costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
        path.push_back(iter_pos);

        while (iter_index != start_index) {
            iter_index = parent_.at(iter_index);
            costmap_ptr_->GetCostMap()->Index2Cells(iter_index, iter_x, iter_y);
            costmap_ptr_->GetCostMap()->Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
            path.push_back(iter_pos);
        }

        std::reverse(path.begin(), path.end());

        return ErrorInfo(ErrorCode::OK);

    }

    ErrorInfo AStarPlanner::GetMoveCost(const int &current_index,
                                        const int &neighbor_index,
                                        int &move_cost) const {
        if (abs(neighbor_index - current_index) == 1 ||
            abs(neighbor_index - current_index) == gridmap_width_) {
            move_cost = 10;
        } else if (abs(neighbor_index - current_index) == (gridmap_width_ + 1) ||
                   abs(neighbor_index - current_index) == (gridmap_width_ - 1)) {
            move_cost = 14;
        } else {
            return ErrorInfo(ErrorCode::GP_MOVE_COST_ERROR,
                             "Move cost can't be calculated cause current neighbor index is not accessible");
        }
        return ErrorInfo(ErrorCode::OK);
    }

    void AStarPlanner::GetManhattanDistance(const int &index1, const int &index2, int &manhattan_distance) const {
        manhattan_distance = heuristic_factor_ * 10 * (abs(index1 / gridmap_width_ - index2 / gridmap_width_) +
                                                       abs(index1 % gridmap_width_ - index2 % gridmap_width_));
    }

    void AStarPlanner::GetNineNeighbors(const int &current_index, std::vector<int> &neighbors_index) const {
        neighbors_index.clear();
        if (current_index - gridmap_width_ >= 0) {
            neighbors_index.push_back(current_index - gridmap_width_);       //up
        }
        if (current_index - gridmap_width_ - 1 >= 0 && (current_index - gridmap_width_ - 1 + 1) % gridmap_width_ != 0) {
            neighbors_index.push_back(current_index - gridmap_width_ - 1); //left_up
        }
        if (current_index - 1 >= 0 && (current_index - 1 + 1) % gridmap_width_ != 0) {
            neighbors_index.push_back(current_index - 1);        //left
        }
        if (current_index + gridmap_width_ - 1 < gridmap_width_ * gridmap_height_
            && (current_index + gridmap_width_ - 1 + 1) % gridmap_width_ != 0) {
            neighbors_index.push_back(current_index + gridmap_width_ - 1); //left_down
        }
        if (current_index + gridmap_width_ < gridmap_width_ * gridmap_height_) {
            neighbors_index.push_back(current_index + gridmap_width_);     //down
        }
        if (current_index + gridmap_width_ + 1 < gridmap_width_ * gridmap_height_
            && (current_index + gridmap_width_ + 1) % gridmap_width_ != 0) {
            neighbors_index.push_back(current_index + gridmap_width_ + 1); //right_down
        }
        if (current_index + 1 < gridmap_width_ * gridmap_height_
            && (current_index + 1) % gridmap_width_ != 0) {
            neighbors_index.push_back(current_index + 1);                   //right
        }
        if (current_index - gridmap_width_ + 1 >= 0
            && (current_index - gridmap_width_ + 1) % gridmap_width_ != 0) {
            neighbors_index.push_back(current_index - gridmap_width_ + 1); //right_up
        }
    }
}