
#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_goal");
  ros::NodeHandle nh("~");

  //【发布】目标点
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/prometheus/planning/goal", 10);

  float x,y,z;

  geometry_msgs::PoseStamped goal;
  int flag;
  cout << "Please choose 2D or 3D (0 for 2D, 1 for 3D):"<<endl;
  cin >> flag;  

  tf::TransformListener* tfListener = new tf::TransformListener();;

  while(ros::ok())
  {
    // Waiting for input
    cout << "Please input the goal position in body frame:"<<endl;
    cout << "goal - x [m] : "<< endl;
    cin >> x;
    cout << "goal -  y [m] : "<< endl;
    cin >> y;
    if(flag == 1)
    {
      cout << "goal -  z [m] : "<< endl;
      cin >> z;
    }else if(flag == 0)
    {
      z = 0.0;
    }

    // point in body frame
    geometry_msgs::PointStamped p_body;
    p_body.header.frame_id = "base_link";
    p_body.header.stamp = ros::Time::now();
    p_body.point.x = x;
    p_body.point.y = y;
    p_body.point.z = z;

    // goal point in map frame
    goal.header.stamp =ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    try
    {
      if (tfListener->waitForTransform("world", "base_link", ros::Time::now(), ros::Duration(1.0)))
      {
        ROS_INFO("Transform Available");
        geometry_msgs::PointStamped p_map;
        tfListener->transformPoint("world", p_body, p_map);
        cout << "goal in map frame: " << p_map.point.x << " [m] "<< p_map.point.y << " [m] "<< p_map.point.z << " [m] "<< endl;
        goal.pose.position.x = p_map.point.x;
        goal.pose.position.y = p_map.point.y;
        goal.pose.position.z = p_map.point.z;
        goal_pub.publish(goal);
      }
      else
      {
        ROS_WARN("Transform Not Available");
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    sleep(1.0);
  }
  
  return 0;
}
