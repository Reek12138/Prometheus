#include <ros/ros.h>
#include "prometheus_msgs/ControlCommand.h"
#include "std_msgs/Header.h"
#include <cmath> // 引入 std::round

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_command_publisher_once");
    ros::NodeHandle nh;

    ros::Publisher control_command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 给ROS一些时间来建立连接
    ros::Duration(0.5).sleep();

    prometheus_msgs::ControlCommand command;
// 1、解锁
    // 设置消息头
    // command.header.seq = 0;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = "";

    command.Command_ID = 1;
    command.source = "terminal_control";
    command.Mode = 0;

    // 设置Reference_State
    command.Reference_State.header.seq = 0;
    command.Reference_State.header.stamp = ros::Time(0, 0);
    command.Reference_State.header.frame_id = "";
    command.Reference_State.Move_mode = 0;
    command.Reference_State.Move_frame = 0;
    command.Reference_State.time_from_start = 0.0;
    command.Reference_State.position_ref = {0.0, 0.0, 0.0};
    command.Reference_State.velocity_ref = {0.0, 0.0, 0.0};
    command.Reference_State.acceleration_ref = {0.0, 0.0, 0.0};
    command.Reference_State.latitude = 0.0;
    command.Reference_State.longitude = 0.0;
    command.Reference_State.altitude = 0.0;
    command.Reference_State.Yaw_Rate_Mode = false;
    command.Reference_State.yaw_ref = 999.0;
    command.Reference_State.yaw_rate_ref = 0.0;

    // 发布消息
    control_command_pub.publish(command);

    // 给ROS时间来处理发布的消息
    ros::Duration(0.5).sleep();

// 2、起飞
     // 设置消息头
    // command.header.seq = 0;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = "";

    command.Command_ID = 2;
    command.source = "terminal_control";
    command.Mode = 1;

    // 设置Reference_State
    command.Reference_State.header.seq = 0;
    command.Reference_State.header.stamp = ros::Time(0, 0);
    command.Reference_State.header.frame_id = "";
    command.Reference_State.Move_mode = 0;
    command.Reference_State.Move_frame = 0;
    command.Reference_State.time_from_start = 0.0;
    command.Reference_State.position_ref = {0.0, 0.0, 0.0};
    command.Reference_State.velocity_ref = {0.0, 0.0, 0.0};
    command.Reference_State.acceleration_ref = {0.0, 0.0, 0.0};
    command.Reference_State.latitude = 0.0;
    command.Reference_State.longitude = 0.0;
    command.Reference_State.altitude = 0.0;
    command.Reference_State.Yaw_Rate_Mode = false;
    command.Reference_State.yaw_ref = 0.0;
    command.Reference_State.yaw_rate_ref = 0.0;

    // 发布消息
    control_command_pub.publish(command);

    // 给ROS时间来处理发布的消息
    ros::Duration(2).sleep();

    double radius = 2.0; // 圆的半径，单位：米
    double omega = 0.5; // 角速度，单位：弧度/秒
    int command_id = 3; // Start counting from 3

    // while(ros::ok()){

    // double t = ros::Time::now().toSec(); // 当前时间，秒
    // double theta = omega * t;
    // float vx = -radius * omega * sin(theta);  // x方向速度
    // float vy = radius * omega * cos(theta);// y方向速度
    // float vx_ = std::round(vx*10)/10.0;
    // float vy_ = std::round(vy*10)/10.0;

    //          // 设置消息头
    // // command.header.seq = 0;
    // command.header.stamp = ros::Time::now();
    // command.header.frame_id = "";

    // command.Command_ID = command_id++;
    // command.source = "terminal_control";
    // command.Mode = 4;

    // // 设置Reference_State
    // command.Reference_State.header.seq = 0;
    // command.Reference_State.header.stamp = ros::Time(0, 0);
    // command.Reference_State.header.frame_id = "";
    // command.Reference_State.Move_mode = 2;
    // command.Reference_State.Move_frame = 1;
    // command.Reference_State.time_from_start = -1.0;
    // command.Reference_State.position_ref = {0.0, 0.0, 0.0};
    // command.Reference_State.velocity_ref = {vx_, vy_, 0.0};
    // // command.Reference_State.velocity_ref = {0.5, 0.5, 0.0};
    // command.Reference_State.acceleration_ref = {0.0, 0.0, 0.0};
    // command.Reference_State.latitude = 0.0;
    // command.Reference_State.longitude = 0.0;
    // command.Reference_State.altitude = 0.0;
    // command.Reference_State.Yaw_Rate_Mode = false;
    // command.Reference_State.yaw_ref = 0.0;
    // command.Reference_State.yaw_rate_ref = 0.0;

    // // 发布消息
    // control_command_pub.publish(command);

    // // 给ROS时间来处理发布的消息
    // ros::Duration(0.).sleep();
    // // return 0;
    // }
 
  
}


