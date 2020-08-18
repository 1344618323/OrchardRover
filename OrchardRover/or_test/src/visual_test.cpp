//
// Created by cxn on 2020/6/2.
//


/**以下代码来自：http://wiki.ros.org/rviz/Tutorials**/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    while (ros::ok()) {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = drand48()*10-5;
        marker.pose.position.y = drand48()*10-5;
        marker.pose.position.z = drand48()*10-5;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        // lifetime指定此marker在被自动删除前保留的时间，Duration意味着永久保留
        marker.lifetime = ros::Duration();

        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);

        // Cycle between different shapes
        switch (shape) {
            case visualization_msgs::Marker::CUBE:
                shape = visualization_msgs::Marker::SPHERE;
                break;
            case visualization_msgs::Marker::SPHERE:
                shape = visualization_msgs::Marker::ARROW;
                break;
            case visualization_msgs::Marker::ARROW:
                shape = visualization_msgs::Marker::CYLINDER;
                break;
            case visualization_msgs::Marker::CYLINDER:
                shape = visualization_msgs::Marker::CUBE;
                break;
        }
        r.sleep();
    }
    return 0;
}

//int main(int argc, char **argv) {
//    ros::init(argc, argv, "points_and_lines");
//    ros::NodeHandle n;
//    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
//
//    ros::Rate r(30);
//
//    float f = 0.0;
//    while (ros::ok()) {
//        visualization_msgs::Marker points, line_strip, line_list;
//        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
//        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
//        points.ns = line_strip.ns = line_list.ns = "points_and_lines";
//        points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
//        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
//        points.id = 0;
//        line_strip.id = 1;
//        line_list.id = 2;
//        points.type = visualization_msgs::Marker::POINTS;
//        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
//        line_list.type = visualization_msgs::Marker::LINE_LIST;
//
//        // POINTS markers use x and y scale for width/height respectively
//        points.scale.x = 0.2;
//        points.scale.y = 0.2;
//
//        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
//        line_strip.scale.x = 0.1;
//        line_list.scale.x = 0.1;
//
//        // Points are green
//        points.color.g = 1.0f;
//        points.color.a = 1.0;
//
//        // Line strip is blue
//        line_strip.color.b = 1.0;
//        line_strip.color.a = 1.0;
//
//        // Line list is red
//        line_list.color.r = 1.0;
//        line_list.color.a = 1.0;
//
//        // Create the vertices for the points and lines
//        for (uint32_t i = 0; i < 100; ++i) {
//            float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
//            float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
//
//            geometry_msgs::Point p;
//            p.x = (int32_t) i - 50;
//            p.y = y;
//            p.z = z;
//
//            points.points.push_back(p);
//            line_strip.points.push_back(p);
//
//            // The line list needs two points for each line
//            line_list.points.push_back(p);
//            p.z += 1.0;
//            line_list.points.push_back(p);
//        }
//
////        marker_pub.publish(points);
//        marker_pub.publish(line_strip);
////        marker_pub.publish(line_list);
//
//        r.sleep();
//
//        f += 0.04;
//    }
//    return 0;
//}


//#include <interactive_markers/interactive_marker_server.h>
//
///*(cxn)
// * 基本套路：创建一个 交互对象服务器 InteractiveMarkerServer，用于更新rviz，或者获取从rviz获得的反馈
// * 创建对象 交互对象 InteractiveMarker，这些对象需要加入到 服务器中，插入服务器时配置{交互对象，该交互对象对应的rviz回调函数}
// * 同时每个交互对象会配置 多个InteractiveMarkerControl对象，每个InteractiveMarkerControl对象会作为一个visualization_msgs::Marker对象的控制器
//*/
//
//void processFeedback(
//        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
//    ROS_INFO_STREAM(feedback->marker_name << " is now at "
//                                          << feedback->pose.position.x << ", " << feedback->pose.position.y
//                                          << ", " << feedback->pose.position.z);
//}
//
//int main(int argc, char **argv) {
//    ros::init(argc, argv, "simple_marker");
//
//    // create an interactive marker server on the topic namespace simple_marker
//    interactive_markers::InteractiveMarkerServer server("simple_marker");
//
//    // create an interactive marker for our server
//    visualization_msgs::InteractiveMarker int_marker;
//    int_marker.header.frame_id = "base_link";
//    int_marker.header.stamp = ros::Time::now();
//    int_marker.name = "my_marker";
//    int_marker.description = "Simple 1-DOF Control";
//
//    // create a grey box marker
//    visualization_msgs::Marker box_marker;
//    box_marker.type = visualization_msgs::Marker::CUBE;
//    box_marker.scale.x = 0.45;
//    box_marker.scale.y = 0.45;
//    box_marker.scale.z = 0.45;
//    box_marker.color.r = 0.5;
//    box_marker.color.g = 0.5;
//    box_marker.color.b = 0.5;
//    box_marker.color.a = 1.0;
//
//    // create a non-interactive control which contains the box
//    visualization_msgs::InteractiveMarkerControl box_control;
//    box_control.always_visible = true;
//    box_control.markers.push_back(box_marker);
//
//    // add the control to the interactive marker
//    int_marker.controls.push_back(box_control);
//
//    // create a control which will move the box
//    // this control does not contain any markers,
//    // which will cause RViz to insert two arrows
//    visualization_msgs::InteractiveMarkerControl rotate_control;
//
//    //下面的四元数表示 [sqrt(2)/2,(0,sqrt(2)/2,0)]，表示 R(global-local)
//    rotate_control.orientation.w = 1;
//    rotate_control.orientation.x = 0;
//    rotate_control.orientation.y = 1;
//    rotate_control.orientation.z = 0;
//
//    rotate_control.name = "move_x";//这只是个名字，没啥卵用
//
////    rotate_control.interaction_mode =
////            visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;//让控制器沿localframe的x方向移动，换算到globalframe就是沿着z方向移动
//
//    rotate_control.interaction_mode =
//            visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;//让控制器绕着localframe的x方向旋转，换算到globalframe就是绕着z方向旋转
//
//    // add the control to the interactive marker
//    int_marker.controls.push_back(rotate_control);
//
//    // add the interactive marker to our collection &
//    // tell the server to call processFeedback() when feedback arrives for it
//    server.insert(int_marker, &processFeedback);
//
//    // 'commit' changes and send to all clients
//    server.applyChanges();
//
//    // start the ROS main loop
//    ros::spin();
//
//    return 0;
//}

//#include <interactive_markers/menu_handler.h>
//#include <tf/transform_broadcaster.h>
//#include <tf/tf.h>
//
//using namespace visualization_msgs;
//
//boost::shared_ptr<interactive_markers::InteractiveMarkerServer>server;
//interactive_markers::MenuHandler menu_handler;
//
//// %Tag(Box)%
//Marker makeBox( InteractiveMarker &msg )
//{
//    Marker marker;
//
//    marker.type = Marker::CUBE;
//    marker.scale.x = msg.scale * 0.45;
//    marker.scale.y = msg.scale * 0.45;
//    marker.scale.z = msg.scale * 0.45;
//    marker.color.r = 0.5;
//    marker.color.g = 0.5;
//    marker.color.b = 0.5;
//    marker.color.a = 1.0;
//
//    return marker;
//}
//
//InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
//{
//    InteractiveMarkerControl control;
//    control.always_visible = true;
//    control.markers.push_back( makeBox(msg) );
//    msg.controls.push_back( control );
//
//    return msg.controls.back();
//}
//// %EndTag(Box)%
//
//// %Tag(frameCallback)%
//void frameCallback(const ros::TimerEvent&)
//{
//    static uint32_t counter = 0;
//
//    static tf::TransformBroadcaster br;
//
//    tf::Transform t;
//
//    ros::Time time = ros::Time::now();
//
//    t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
//    t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
//    br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));
//
//    t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//    t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
//    br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));
//
//    counter++;
//}
//// %EndTag(frameCallback)%
//
//// %Tag(processFeedback)%
//void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
//{
//    std::ostringstream s;
//    s << "Feedback from marker '" << feedback->marker_name << "' "
//      << " / control '" << feedback->control_name << "'";
//
//    std::ostringstream mouse_point_ss;
//    if( feedback->mouse_point_valid )
//    {
//        mouse_point_ss << " at " << feedback->mouse_point.x
//                       << ", " << feedback->mouse_point.y
//                       << ", " << feedback->mouse_point.z
//                       << " in frame " << feedback->header.frame_id;
//    }
//
//    switch ( feedback->event_type )
//    {
//        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
//            ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
//            break;
//
//        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
//            ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
//            break;
//
//        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
//            ROS_INFO_STREAM( s.str() << ": pose changed"
//                                     << "\nposition = "
//                                     << feedback->pose.position.x
//                                     << ", " << feedback->pose.position.y
//                                     << ", " << feedback->pose.position.z
//                                     << "\norientation = "
//                                     << feedback->pose.orientation.w
//                                     << ", " << feedback->pose.orientation.x
//                                     << ", " << feedback->pose.orientation.y
//                                     << ", " << feedback->pose.orientation.z
//                                     << "\nframe: " << feedback->header.frame_id
//                                     << " time: " << feedback->header.stamp.sec << "sec, "
//                                     << feedback->header.stamp.nsec << " nsec" );
//            break;
//
//        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
//            ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
//            break;
//
//        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
//            ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
//            break;
//    }
//
//    server->applyChanges();
//}
//// %EndTag(processFeedback)%
//
//// %Tag(alignMarker)%
//void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
//{
//    geometry_msgs::Pose pose = feedback->pose;
//
//    pose.position.x = round(pose.position.x-0.5)+0.5;
//    pose.position.y = round(pose.position.y-0.5)+0.5;
//
//    ROS_INFO_STREAM( feedback->marker_name << ":"
//                                           << " aligning position = "
//                                           << feedback->pose.position.x
//                                           << ", " << feedback->pose.position.y
//                                           << ", " << feedback->pose.position.z
//                                           << " to "
//                                           << pose.position.x
//                                           << ", " << pose.position.y
//                                           << ", " << pose.position.z );
//
//    server->setPose( feedback->marker_name, pose );
//    server->applyChanges();
//}
//// %EndTag(alignMarker)%
//
//double rand( double min, double max )
//{
//    double t = (double)rand() / (double)RAND_MAX;
//    return min + t*(max-min);
//}
//
//void saveMarker( InteractiveMarker int_marker )
//{
//    server->insert(int_marker);
//    server->setCallback(int_marker.name, &processFeedback);
//}
//
//////////////////////////////////////////////////////////////////////////////////////
//
//// %Tag(6DOF)%
//void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
//{
//    InteractiveMarker int_marker;
//    int_marker.header.frame_id = "base_link";
//    tf::pointTFToMsg(position, int_marker.pose.position);
//    int_marker.scale = 1;
//
//    int_marker.name = "simple_6dof";
//    int_marker.description = "Simple 6-DOF Control";
//
//    // insert a box
//    makeBoxControl(int_marker);
//    int_marker.controls[0].interaction_mode = interaction_mode;
//
//    InteractiveMarkerControl control;
//
//    if ( fixed )
//    {
//        int_marker.name += "_fixed";
//        int_marker.description += "\n(fixed orientation)";
//        control.orientation_mode = InteractiveMarkerControl::FIXED;
//    }
//
//    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
//    {
//        std::string mode_text;
//        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
//        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
//        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
//        int_marker.name += "_" + mode_text;
//        int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
//    }
//
//    if(show_6dof)
//    {
//        control.orientation.w = 1;
//        control.orientation.x = 1;
//        control.orientation.y = 0;
//        control.orientation.z = 0;
//        control.name = "rotate_x";
//        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//        int_marker.controls.push_back(control);
//        control.name = "move_x";
//        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
//        int_marker.controls.push_back(control);
//
//        control.orientation.w = 1;
//        control.orientation.x = 0;
//        control.orientation.y = 1;
//        control.orientation.z = 0;
//        control.name = "rotate_z";
//        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//        int_marker.controls.push_back(control);
//        control.name = "move_z";
//        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
//        int_marker.controls.push_back(control);
//
//        control.orientation.w = 1;
//        control.orientation.x = 0;
//        control.orientation.y = 0;
//        control.orientation.z = 1;
//        control.name = "rotate_y";
//        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//        int_marker.controls.push_back(control);
//        control.name = "move_y";
//        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
//        int_marker.controls.push_back(control);
//    }
//
//    server->insert(int_marker);
//    server->setCallback(int_marker.name, &processFeedback);
//    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
//        menu_handler.apply( *server, int_marker.name );
//}
//// %EndTag(6DOF)%
//
//// %Tag(RandomDof)%
//void makeRandomDofMarker( const tf::Vector3& position )
//{
//    InteractiveMarker int_marker;
//    int_marker.header.frame_id = "base_link";
//    tf::pointTFToMsg(position, int_marker.pose.position);
//    int_marker.scale = 1;
//
//    int_marker.name = "6dof_random_axes";
//    int_marker.description = "6-DOF\n(Arbitrary Axes)";
//
//    makeBoxControl(int_marker);
//
//    InteractiveMarkerControl control;
//
//    for ( int i=0; i<3; i++ )
//    {
//        control.orientation.w = rand(-1,1);
//        control.orientation.x = rand(-1,1);
//        control.orientation.y = rand(-1,1);
//        control.orientation.z = rand(-1,1);
//        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//        int_marker.controls.push_back(control);
//        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
//        int_marker.controls.push_back(control);
//    }
//
//    server->insert(int_marker);
//    server->setCallback(int_marker.name, &processFeedback);
//}
//// %EndTag(RandomDof)%
//
//
//// %Tag(ViewFacing)%
//void makeViewFacingMarker( const tf::Vector3& position )
//{
//    InteractiveMarker int_marker;
//    int_marker.header.frame_id = "base_link";
//    tf::pointTFToMsg(position, int_marker.pose.position);
//    int_marker.scale = 1;
//
//    int_marker.name = "view_facing";
//    int_marker.description = "View Facing 6-DOF";
//
//    InteractiveMarkerControl control;
//
//    // make a control that rotates around the view axis
//    control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
//    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//    control.orientation.w = 1;
//    control.name = "rotate";
//
//    int_marker.controls.push_back(control);
//
//    // create a box in the center which should not be view facing,
//    // but move in the camera plane.
//    control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
//    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
//    control.independent_marker_orientation = true;
//    control.name = "move";
//
//    control.markers.push_back( makeBox(int_marker) );
//    control.always_visible = true;
//
//    int_marker.controls.push_back(control);
//
//    server->insert(int_marker);
//    server->setCallback(int_marker.name, &processFeedback);
//}
//// %EndTag(ViewFacing)%
//
//
//// %Tag(Quadrocopter)%
//void makeQuadrocopterMarker( const tf::Vector3& position )
//{
//    InteractiveMarker int_marker;
//    int_marker.header.frame_id = "base_link";
//    tf::pointTFToMsg(position, int_marker.pose.position);
//    int_marker.scale = 1;
//
//    int_marker.name = "quadrocopter";
//    int_marker.description = "Quadrocopter";
//
//    makeBoxControl(int_marker);
//
//    InteractiveMarkerControl control;
//
//    control.orientation.w = 1;
//    control.orientation.x = 0;
//    control.orientation.y = 1;
//    control.orientation.z = 0;
//    control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
//    int_marker.controls.push_back(control);
//    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
//    int_marker.controls.push_back(control);
//
//    server->insert(int_marker);
//    server->setCallback(int_marker.name, &processFeedback);
//}
//// %EndTag(Quadrocopter)%
//
//// %Tag(ChessPiece)%
//void makeChessPieceMarker( const tf::Vector3& position )
//{
//    InteractiveMarker int_marker;
//    int_marker.header.frame_id = "base_link";
//    tf::pointTFToMsg(position, int_marker.pose.position);
//    int_marker.scale = 1;
//
//    int_marker.name = "chess_piece";
//    int_marker.description = "Chess Piece\n(2D Move + Alignment)";
//
//    InteractiveMarkerControl control;
//
//    control.orientation.w = 1;
//    control.orientation.x = 0;
//    control.orientation.y = 1;
//    control.orientation.z = 0;
//    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
//    int_marker.controls.push_back(control);
//
//    // make a box which also moves in the plane
//    control.markers.push_back( makeBox(int_marker) );
//    control.always_visible = true;
//    int_marker.controls.push_back(control);
//
//    // we want to use our special callback function
//    server->insert(int_marker);
//    server->setCallback(int_marker.name, &processFeedback);
//
//    // set different callback for POSE_UPDATE feedback
//    server->setCallback(int_marker.name, &alignMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
//}
//// %EndTag(ChessPiece)%
//
//// %Tag(PanTilt)%
//void makePanTiltMarker( const tf::Vector3& position )
//{
//    InteractiveMarker int_marker;
//    int_marker.header.frame_id = "base_link";
//    tf::pointTFToMsg(position, int_marker.pose.position);
//    int_marker.scale = 1;
//
//    int_marker.name = "pan_tilt";
//    int_marker.description = "Pan / Tilt";
//
//    makeBoxControl(int_marker);
//
//    InteractiveMarkerControl control;
//
//    control.orientation.w = 1;
//    control.orientation.x = 0;
//    control.orientation.y = 1;
//    control.orientation.z = 0;
//    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//    control.orientation_mode = InteractiveMarkerControl::FIXED;
//    int_marker.controls.push_back(control);
//
//    control.orientation.w = 1;
//    control.orientation.x = 0;
//    control.orientation.y = 0;
//    control.orientation.z = 1;
//    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//    control.orientation_mode = InteractiveMarkerControl::INHERIT;
//    int_marker.controls.push_back(control);
//
//    server->insert(int_marker);
//    server->setCallback(int_marker.name, &processFeedback);
//}
//// %EndTag(PanTilt)%
//
//// %Tag(Menu)%
//void makeMenuMarker( const tf::Vector3& position )
//{
//    InteractiveMarker int_marker;
//    int_marker.header.frame_id = "base_link";
//    tf::pointTFToMsg(position, int_marker.pose.position);
//    int_marker.scale = 1;
//
//    int_marker.name = "context_menu";
//    int_marker.description = "Context Menu\n(Right Click)";
//
//    InteractiveMarkerControl control;
//
//    control.interaction_mode = InteractiveMarkerControl::MENU;
//    control.name = "menu_only_control";
//
//    Marker marker = makeBox( int_marker );
//    control.markers.push_back( marker );
//    control.always_visible = true;
//    int_marker.controls.push_back(control);
//
//    server->insert(int_marker);
//    server->setCallback(int_marker.name, &processFeedback);
//    menu_handler.apply( *server, int_marker.name );
//}
//// %EndTag(Menu)%
//
//// %Tag(Button)%
//void makeButtonMarker( const tf::Vector3& position )
//{
//    InteractiveMarker int_marker;
//    int_marker.header.frame_id = "base_link";
//    tf::pointTFToMsg(position, int_marker.pose.position);
//    int_marker.scale = 1;
//
//    int_marker.name = "button";
//    int_marker.description = "Button\n(Left Click)";
//
//    InteractiveMarkerControl control;
//
//    control.interaction_mode = InteractiveMarkerControl::BUTTON;
//    control.name = "button_control";
//
//    Marker marker = makeBox( int_marker );
//    control.markers.push_back( marker );
//    control.always_visible = true;
//    int_marker.controls.push_back(control);
//
//    server->insert(int_marker);
//    server->setCallback(int_marker.name, &processFeedback);
//}
//// %EndTag(Button)%
//
//// %Tag(Moving)%
//void makeMovingMarker( const tf::Vector3& position )
//{
//    InteractiveMarker int_marker;
//    int_marker.header.frame_id = "moving_frame";
//    tf::pointTFToMsg(position, int_marker.pose.position);
//    int_marker.scale = 1;
//
//    int_marker.name = "moving";
//    int_marker.description = "Marker Attached to a\nMoving Frame";
//
//    InteractiveMarkerControl control;
//
//    control.orientation.w = 1;
//    control.orientation.x = 1;
//    control.orientation.y = 0;
//    control.orientation.z = 0;
//    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//    int_marker.controls.push_back(control);
//
//    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
//    control.always_visible = true;
//    control.markers.push_back( makeBox(int_marker) );
//    int_marker.controls.push_back(control);
//
//    server->insert(int_marker);
//    server->setCallback(int_marker.name, &processFeedback);
//}
//// %EndTag(Moving)%
//
//int main(int argc, char **argv) {
//    ros::init(argc, argv, "basic_controls");
//    ros::NodeHandle n;
//
//// create a timer to update the published transforms
//    ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);
//
//    server.reset(new interactive_markers::InteractiveMarkerServer("basic_controls", "", false));
//
//    ros::Duration(0.1).sleep();
//
//    menu_handler.insert("First Entry", &processFeedback);
//    menu_handler.insert("Second Entry", &processFeedback);
//    interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert("Submenu");
//    menu_handler.insert(sub_menu_handle, "First Entry", &processFeedback);
//    menu_handler.insert(sub_menu_handle, "Second Entry", &processFeedback);
//
//    tf::Vector3 position;
//    position = tf::Vector3(-3, 3, 0);
//    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
////    position = tf::Vector3( 0, 3, 0);
////    make6DofMarker( true, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
////    position = tf::Vector3( 3, 3, 0);
////    makeRandomDofMarker( position );
////    position = tf::Vector3(-3, 0, 0);
////    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::ROTATE_3D, position, false );
////    position = tf::Vector3( 0, 0, 0);
////    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );
////    position = tf::Vector3( 3, 0, 0);
////    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_3D, position, false );
////    position = tf::Vector3(-3,-3, 0);
////    makeViewFacingMarker( position );
////    position = tf::Vector3( 0,-3, 0);
////    makeQuadrocopterMarker( position );
////    position = tf::Vector3( 3,-3, 0);
////    makeChessPieceMarker( position );
////    position = tf::Vector3(-3,-6, 0);
////    makePanTiltMarker( position );
////    position = tf::Vector3( 0,-6, 0);
////    makeMovingMarker( position );
////    position = tf::Vector3( 3,-6, 0);
////    makeMenuMarker( position );
////    position = tf::Vector3( 0,-9, 0);
////    makeButtonMarker( position );
////
//    server->applyChanges();
//
//    ros::spin();
//
//    server.reset();
//
//    return 0;
//}