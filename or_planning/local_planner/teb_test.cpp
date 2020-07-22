//
// Created by cxn on 2020/7/21.
//

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include "include/obstacle.h"
#include "include/local_visualization.h"
#include "include/robot_footprint_model.h"

#include "timed_elastic_band/include/teb_optimal.h"
#include "timed_elastic_band/proto/timed_elastic_band.pb.h"

using namespace or_local_planner;

TebOptimalPtr planner;
std::vector<ObstaclePtr> obst_vector;
LocalVisualizationPtr visual;
ViaPointContainer via_points;
unsigned int no_fixed_obstacles;


void CB_mainCycle(const ros::TimerEvent &e);

void CB_publishCycle(const ros::TimerEvent &e);

void CreateInteractiveMarker(const double &init_x, const double &init_y, unsigned int id, std::string frame,
                             interactive_markers::InteractiveMarkerServer *marker_server,
                             interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_optim_node");
    ros::NodeHandle nh;

    or_local_planner::Config param_config_;

    std::string full_path = ros::package::getPath("or_planning") +
                            "/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt";
    or_common::ReadProtoFromTextFile(full_path.c_str(), &param_config_);

    ros::Timer cycle_timer = nh.createTimer(ros::Duration(0.025), CB_mainCycle);
    ros::Timer publish_timer = nh.createTimer(ros::Duration(0.1), CB_publishCycle);


    interactive_markers::InteractiveMarkerServer marker_server("marker_obstacles");

    obst_vector.emplace_back(std::make_shared<PointObstacle>(-3, 1));
    obst_vector.emplace_back(std::make_shared<PointObstacle>(6, 2));
    obst_vector.emplace_back(std::make_shared<PointObstacle>(0, 0.1));
    obst_vector.emplace_back(std::make_shared<PointObstacle>(-4, 1));
    obst_vector.emplace_back(std::make_shared<PointObstacle>(5, 2));
    obst_vector.emplace_back(std::make_shared<PointObstacle>(1, 0.1));
    obst_vector.emplace_back(std::make_shared<PointObstacle>(-3, 2));
    obst_vector.emplace_back(std::make_shared<PointObstacle>(5, 3));
    obst_vector.emplace_back(std::make_shared<PointObstacle>(4, 0));
    obst_vector.emplace_back(std::make_shared<PointObstacle>(4, 1));
    obst_vector.emplace_back(std::make_shared<PointObstacle>(3, 2));
    obst_vector.emplace_back(std::make_shared<PointObstacle>(2, 2));

    std::string map_frame;
    for (unsigned int i = 0; i < obst_vector.size(); ++i) {

        std::shared_ptr<PointObstacle> pobst = std::dynamic_pointer_cast<PointObstacle>(obst_vector.at(i));
        if (pobst) {
            CreateInteractiveMarker(pobst->Position().coeff(0), pobst->Position().coeff(1),
                                    i, "odom", &marker_server, &CB_obstacle_marker);
        }
    }
    marker_server.applyChanges();

    // Setup visualization
    visual = LocalVisualizationPtr(new LocalVisualization(nh, "odom"));

    RobotFootprintModelPtr model = std::make_shared<PointRobotFootprint>();

    planner = TebOptimalPtr(new TebOptimal(param_config_, &obst_vector, model, visual, &via_points));

    no_fixed_obstacles = (unsigned int) obst_vector.size();

    ros::spin();

    return 0;
}


void CreateInteractiveMarker(const double &init_x, const double &init_y, unsigned int id, std::string frame,
                             interactive_markers::InteractiveMarkerServer *marker_server,
                             interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb) {
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker i_marker;
    i_marker.header.frame_id = frame;
    i_marker.header.stamp = ros::Time::now();
    std::ostringstream oss;
    //oss << "obstacle" << id;
    oss << id;
    i_marker.name = oss.str();
    i_marker.description = "Obstacle";
    i_marker.pose.position.x = init_x;
    i_marker.pose.position.y = init_y;

    // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.id = id;
    box_marker.scale.x = 0.2;
    box_marker.scale.y = 0.2;
    box_marker.scale.z = 0.2;
    box_marker.color.r = 100;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = 1;
    box_control.markers.push_back(box_marker);

    // add the control to the interactive marker
    i_marker.controls.push_back(box_control);

    // create a control which will move the box, rviz will insert 2 arrows
    visualization_msgs::InteractiveMarkerControl move_control;
    move_control.name = "move_x";
    move_control.orientation.w = sqrt(2) / 2;
    move_control.orientation.x = 0;
    move_control.orientation.y = sqrt(2) / 2;
    move_control.orientation.z = 0;
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


    // add the control to the interactive marker
    i_marker.controls.push_back(move_control);

    // add the interactive marker to our collection
    marker_server->insert(i_marker);
    marker_server->setCallback(i_marker.name, feedback_cb);
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent &e) {
    auto start_pose = DataConverter::LocalConvertCData(-4, 0, 0);
    auto end_pose = DataConverter::LocalConvertCData(4, 0, 0);
    planner->Optimal(DataBase(start_pose.first, start_pose.second), DataBase(end_pose.first, end_pose.second));
}

// Visualization loop
void CB_publishCycle(const ros::TimerEvent &e) {
    planner->Visualize();
}

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    std::stringstream ss(feedback->marker_name);
    unsigned int index;
    ss >> index;
    if (index >= no_fixed_obstacles)
        return;
    PointObstacle *pobst = dynamic_cast<PointObstacle *>(obst_vector.at(index).get());
    pobst->Position() = Eigen::Vector2d(feedback->pose.position.x, feedback->pose.position.y);
}