#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "or_msgs/TrunkObsMsg.h"

void callback(const or_msgs::TrunkObsMsg::ConstPtr &msg1, const or_msgs::TrunkObsMsg::ConstPtr &msg2) {
    std::cout << "start" << std::endl;
    for (int i = 0; i < msg1->ranges.size(); i++) {
        std::cout << msg1->ranges[i] << ",";
    }
    std::cout << std::endl;
    for (int i = 0; i < msg2->ranges.size(); i++) {
        std::cout << msg2->ranges[i] << ",";
    }
    std::cout << std::endl;
    std::cout << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "timesynchronizer_node");

    ros::NodeHandle nh;

    message_filters::Subscriber <or_msgs::TrunkObsMsg> image_sub(nh, "chatter", 1);
    message_filters::Subscriber <or_msgs::TrunkObsMsg> info_sub(nh, "chatter2", 1);
    // message_filters::TimeSynchronizer<M0~M8>继承自message_filters::Synchronizer<sync_policies::ExactTime<M0~M8>>
    // 必须时间戳完全一样才能跑回调
    // message_filters::TimeSynchronizer<or_msgs::TrunkObsMsg, or_msgs::TrunkObsMsg> sync(image_sub, info_sub, 10);

    typedef message_filters::sync_policies::ApproximateTime <or_msgs::TrunkObsMsg, or_msgs::TrunkObsMsg> MySyncPolicy;
    message_filters::Synchronizer <MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}
