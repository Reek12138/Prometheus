#include "xsugv_team_center/ros_master_relay/ros_master_relay.h"
#include <ros/spinner.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_master_relay_node");
    xsugv::RosMasterRelay ros_master_relay;
    // 有service存在, 使用多线程
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
//    ros::spin();
    return 0;
}
