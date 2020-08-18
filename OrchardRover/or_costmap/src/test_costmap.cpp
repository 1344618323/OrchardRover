//
// Created by cxn on 2020/7/3.
//

#include "costmap_interface.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "test_costmap", ros::init_options::NoSigintHandler);
    tf::TransformListener tf(ros::Duration(10));

    std::string local_map = ros::package::getPath("or_costmap") + \
      "/config/costmap_parameter_config_for_local_plan.prototxt";
    or_costmap::CostmapInterface costmap_interface("map",
                                                   tf,
                                                   local_map.c_str());

    ros::spin();
    return 0;
}