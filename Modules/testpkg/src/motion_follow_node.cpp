#include <ros/ros.h>
#include <iostream>

#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/SwarmCommand.h>
#include <geometry_msgs/PoseStamped.h>

prometheus_msgs::SwarmCommand swarm_command_to_pub;
ros::Publisher command_pub;
ros::Publisher command_vis_pub;

void state_cb(const prometheus_msgs::DroneState::ConstPtr &msg);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_follow");
  ros::NodeHandle nh("~");

  int followed_uav_id, follower_uav_id;
  nh.param("followed_uav_id", followed_uav_id, 1);  // followed
  nh.param("follower_uav_id", follower_uav_id, 2);  // follower

  ROS_INFO("motion_follow start");

  std::string followed_uav_name = "/uav" + std::to_string(followed_uav_id);
  std::string follower_uav_name = "/uav" + std::to_string(follower_uav_id);

  ros::Subscriber state_sub = nh.subscribe<prometheus_msgs::DroneState>(followed_uav_name + "/prometheus/drone_state", 10, state_cb);
  command_pub = nh.advertise<prometheus_msgs::SwarmCommand>(follower_uav_name + "/prometheus/swarm_command", 10);
  command_vis_pub = nh.advertise<geometry_msgs::PoseStamped>(follower_uav_name + "/prometheus/swarm_command_vis", 10);

  swarm_command_to_pub.Mode = prometheus_msgs::SwarmCommand::Idle;
  swarm_command_to_pub.Command_ID = 0;
  swarm_command_to_pub.source = "Follower";
  swarm_command_to_pub.Move_mode = prometheus_msgs::SwarmCommand::XYZ_POS;
  swarm_command_to_pub.position_ref[0] = 0.0;
  swarm_command_to_pub.position_ref[1] = 0.0;
  swarm_command_to_pub.position_ref[2] = 0.0;
  swarm_command_to_pub.yaw_ref = 0.0;

  ros::spin();

  return 0;
}

void state_cb(const prometheus_msgs::DroneState::ConstPtr &msg)
{
  if (msg->connected == false || msg->armed == false)
  {
    ROS_INFO("Drone is not ready!");
    return;
  }

  swarm_command_to_pub.header.stamp = ros::Time::now();
  swarm_command_to_pub.Mode = prometheus_msgs::SwarmCommand::Move;
  swarm_command_to_pub.Command_ID += 1;
  swarm_command_to_pub.source = "Follower";
  swarm_command_to_pub.Move_mode = prometheus_msgs::SwarmCommand::XYZ_POS;
  swarm_command_to_pub.position_ref[0] = msg->position[0]-1.0;
  swarm_command_to_pub.position_ref[1] = msg->position[1];
  swarm_command_to_pub.position_ref[2] = msg->position[2];
  swarm_command_to_pub.yaw_ref = msg->attitude[2];

  command_pub.publish(swarm_command_to_pub);

  geometry_msgs::PoseStamped command_vis;
  command_vis.header.stamp = ros::Time::now();
  command_vis.header.frame_id = "world";
  command_vis.pose.position.x = msg->position[0]-1.0;
  command_vis.pose.position.y = msg->position[1];
  command_vis.pose.position.z = msg->position[2];
  command_vis.pose.orientation = msg->attitude_q;
  command_vis_pub.publish(command_vis);
}