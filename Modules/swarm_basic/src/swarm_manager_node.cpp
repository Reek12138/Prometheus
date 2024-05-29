/**
 * @file swarm_manager_node.cpp
 * @author Jialong Zeng
 * @brief 将无人机状态信息转换到同一坐标系下，将同一坐标系下的控制指令转换到各个无人机的控制指令
 * 				无人编号从 1 开始，1 号是主机
 * @version 0.1
 * @date 2024-05-07
 * 
 * @todo 1. 系统内编号和飞控编号
 * 			 2. 指令信息转换部分和测试 (指令发布者已注释掉)
 * 			 3. 话题名, 在launch里面remap
 * 			 4. 弃用timer，改为订阅无人机状态信息，有新信息时发布可视化
 * 
 */

/* sys include */
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <boost/bind.hpp>

/* basic pkg include */
#include <geometry_msgs/PoseStamped.h>

/* external pkg include */
#include "prometheus_msgs/DroneState.h"
#include "prometheus_msgs/SwarmCommand.h"

using namespace Eigen;

/* variables */
int swarm_num; // 集群内无人机数量
std::vector<Vector3d> t_i2o; // 坐标原点相对主坐标系的偏移量
std::vector<ros::Subscriber> drone_state_subs; // 订阅所有无人机状态信息
std::vector<ros::Subscriber> drone_command_subs; // 订阅所有无人机控制指令(主机坐标系下)
std::vector<ros::Publisher> drone_command_pubs; // 发布所有无人机控制指令
std::vector<ros::Publisher> drone_state_vis_pubs; // 发布所有无人机状态信息(仅用于可视化)
std::vector<ros::Publisher> drone_state_pubs; // 发布所有无人机状态信息
std::vector<prometheus_msgs::DroneState> drone_states; // 所有无人机状态信息
ros::Timer drone_state_vis_timer; // 定时器

/* functions */
/**
 * @brief 接收飞控发来的无人机状态信息，转换到主机坐标系下
 * 
 * @param msg 
 * @param id 
 */
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr &msg, int id) {
	prometheus_msgs::DroneState state = *msg;
	// state.connected = state.connected;
	// state.armed = state.armed;
	// state.landed = state.landed;
	// state.mode = state.mode;
	// state.odom_valid = state.odom_valid;
	// state.time_from_start = state.time_from_start;
	state.position[0] = state.position[0] + t_i2o[id][0];
	state.position[1] = state.position[1] + t_i2o[id][1];
	state.position[2] = state.position[2] + t_i2o[id][2];
	// state.rel_alt = state.rel_alt;
	// state.velocity = state.velocity;
	// state.attitude = state.attitude;
	// state.attitude_q = state.attitude_q;
	// state.attitude_rate = state.attitude_rate;
	drone_states[id] = state;
	drone_state_pubs[id].publish(state);
}

/**
 * @brief 接收无人机指令(主机坐标系下)，转换到各个无人机坐标系下，再发布
 * 				!! 注意：还未测试
 * 
 * @param msg 
 * @param id 
 */
void drone_command_cb(const prometheus_msgs::SwarmCommand::ConstPtr &msg, int id) {
	prometheus_msgs::SwarmCommand command = *msg;
	// command.header = command.header;
	// command.Mode = command.Mode;
	command.position_ref[0] = command.position_ref[0] - t_i2o[id][0];
	command.position_ref[1] = command.position_ref[1] - t_i2o[id][1];
	command.position_ref[2] = command.position_ref[2] - t_i2o[id][2];
	// command.velocity_ref = command.velocity_ref;
	// command.acceleration_ref = command.acceleration_ref;
	// command.yaw_ref = command.yaw_ref;
	// command.yaw_rate_ref = command.yaw_rate_ref;
	drone_command_pubs[id].publish(command);
}

/**
 * @brief 定时发布所有无人机状态信息可视化
 * 
 * @param event 
 */
void drone_state_vis_timer_cb(const ros::TimerEvent &event) {
	geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();
	for (int i = 0; i < swarm_num; i++) {
		if (drone_states[i].connected) {
			pose.header.stamp = drone_states[i].header.stamp;
			pose.header.frame_id = "world";
			pose.pose.position.x = drone_states[i].position[0];
			pose.pose.position.y = drone_states[i].position[1];
			pose.pose.position.z = drone_states[i].position[2];
			pose.pose.orientation.w = drone_states[i].attitude_q.w;
			pose.pose.orientation.x = drone_states[i].attitude_q.x;
			pose.pose.orientation.y = drone_states[i].attitude_q.y;
			pose.pose.orientation.z = drone_states[i].attitude_q.z;
			drone_state_vis_pubs[i].publish(pose);
		}
	}
}

/* main function */
int main(int argc, char **argv) {
	/* node init */
	ros::init(argc, argv, "swarm_manager_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(20.0);

	/* params */
	nh.param<int>("swarm_num", swarm_num, 1);
	t_i2o.push_back(Vector3d::Zero()); // 主机坐标系，无偏移
	for (int i = 1; i < swarm_num; i++) {
		Vector3d t(0.0, 0.0, 0.0);
		nh.param<double>("offset_x_" + std::to_string(i+1), t[0], 0.0);
		nh.param<double>("offset_y_" + std::to_string(i+1), t[1], 0.0);
		nh.param<double>("offset_z_" + std::to_string(i+1), t[2], 0.0);
		t_i2o.push_back(t);
	}

	/* variables init */
	for (int i = 0; i < swarm_num; i++) {
		prometheus_msgs::DroneState state = prometheus_msgs::DroneState();
		state.connected = false;
		drone_states.push_back(state);
	}

	/* subscribers */
	for (int i = 0; i < swarm_num; i++) {
		ros::Subscriber sub = nh.subscribe<prometheus_msgs::DroneState>(
			"/uav" + std::to_string(i+1) + "/prometheus/drone_state",
			10,
			boost::bind(drone_state_cb, _1, i));
		drone_state_subs.push_back(sub);
	}
	for (int i = 0; i < swarm_num; i++) {
		ros::Subscriber sub = nh.subscribe<prometheus_msgs::SwarmCommand>(
			"/uav" + std::to_string(i+1) + "/prometheus/swarm_command_manager",
			10,
			boost::bind(drone_command_cb, _1, i));
		drone_command_subs.push_back(sub);
	}

	/* publishers */
	for (int i = 0; i < swarm_num; i++) {
		ros::Publisher pub = nh.advertise<prometheus_msgs::SwarmCommand>(
			"/uav" + std::to_string(i+1) + "/prometheus/swarm_command",
			10);
		drone_command_pubs.push_back(pub);
	}
	for (int i = 0; i < swarm_num; i++) {
		ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>(
			"/uav" + std::to_string(i+1) + "/prometheus/drone_state_vis",
			10);
		drone_state_vis_pubs.push_back(pub);
	}
	for (int i = 0; i < swarm_num; i++) {
		ros::Publisher pub = nh.advertise<prometheus_msgs::DroneState>(
			"/uav" + std::to_string(i+1) + "/prometheus/drone_state_manager",
			10);
		drone_state_pubs.push_back(pub);
	}

	/* timer */
	drone_state_vis_timer = nh.createTimer(ros::Duration(0.05), drone_state_vis_timer_cb);

	/* main loop */
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
