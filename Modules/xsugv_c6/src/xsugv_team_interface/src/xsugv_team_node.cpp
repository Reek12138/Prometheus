#include <ros/ros.h>
#include "xsugv_team_interface/xsugv_team_interface.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "xsugv_team_node");
  ros::NodeHandle nh, private_nh("~");

  xsugv::XSUGVTeamInterface iface(nh, private_nh);

  ros::spin();

  return 0;
}

