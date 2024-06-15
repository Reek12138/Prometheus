/**
 * @file finite_time_tracking_node.cpp
 * @author Jialong Zeng (colenz at qq dot com)
 * @brief 在Prometheus平台实现有限时间轨迹跟踪
 * @version 0.1
 * @date 2024-05-14
 * 
 * @note 已经调到了较优的参数，关键在于权重k和速度正则化，让无人机能跟上轨迹 
 * 
 */

/* sys include */
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <Eigen/Eigen>

/* external pkg */
#include "prometheus_msgs/ControlCommand.h"
#include "prometheus_msgs/DroneState.h"
#include "prometheus_msgs/PositionReference.h"

/* variable */
ros::Timer track_timer;
ros::Subscriber trigger_sub;
ros::Subscriber state_sub;
ros::Publisher control_pub;
ros::Publisher refpath_pub;
ros::Publisher vel_err_pub;
prometheus_msgs::ControlCommand control_command;
prometheus_msgs::DroneState drone_state;
visualization_msgs::Marker ref_path;
geometry_msgs::Vector3Stamped vel_err;
bool track_flag;

/* util functions */
int sgn(const double x);
bool satisfiesEquations(const Eigen::Vector3d _pos, const double _r, const double _offset);

/* tracking compute functions */
void computeRefPath(visualization_msgs::Marker& _path, const double _r, const double _offset = 0.0) {
	visualization_msgs::Marker points;
	points.header.frame_id = "world";
	points.header.stamp = ros::Time::now();
	points.ns = "points";
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.action = visualization_msgs::Marker::ADD;
	points.scale.x = 0.03;
	points.scale.y = 0.03;
	points.color.r = 1.0;
	points.color.g = 0.0;
	points.color.b = 0.0;
	points.color.a = 1.0;

	double delta = 0.02;

	for (double z = 0 + _offset; z <= 2.5 + _offset; z += delta) {
		for (double y = -2; y <= 2; y += delta) {
			for (double x = -2; x <= 2; x += delta) {
				Eigen::Vector3d pos = Eigen::Vector3d::Zero();
				pos[0] = x;
				pos[1] = y;
				pos[2] = z;
				if (satisfiesEquations(pos, _r, _offset)) {
					geometry_msgs::Point p;
					p.x = x;
					p.y = y;
					p.z = z;
					points.points.push_back(p);
				}
			}
		}
	}
	_path = points;
}

/**
 * @brief 根据当前位置计算期望速度
 * 
 * @param pos_ref 
 * @param vel_ref 
 * @return true 
 * @return false 
 */
Eigen::Vector3d computeRefVel(const Eigen::Vector3d _pos) {
	/* params */
	double r = 1.0;
	double k1 = 1.0;
	double k2 = 1.0;
	double alpha = 0.5;
	double z_offset = 5.0;

	/* variables */
	double x = _pos[0];
	double y = _pos[1];
	double z = _pos[2] - z_offset;
	Eigen::Vector3d n1 = Eigen::Vector3d::Zero();
	Eigen::Vector3d n2 = Eigen::Vector3d::Zero();
	double e1, e2;
	double sig_e1_alpha, sig_e2_alpha;

	// n1 = [2x, 0, 2(z-3/2)]
	n1[0] = 2 * x;
	n1[1] = 0;
	n1[2] = 2 * (z - 1.5);

	// n2 = [0, 2y, 2z]
	n2[0] = 0;
	n2[1] = 2 * y;
	n2[2] = 2 * z;

	// e1 = x^2 + (z-3/2)^2 - r^2
	e1 = x * x + (z - 1.5) * (z - 1.5) - r * r;
	
	// e2 = y^2 + z^2 - 4
	e2 = y * y + z * z - 4;

	// sig_e1_alpha & sig_e2_alpha
	sig_e1_alpha = sgn(e1) * pow(abs(e1), alpha);
	sig_e2_alpha = sgn(e2) * pow(abs(e2), alpha);

	// vel_ref = n1 x n2 - [ (k1 * sig(e1)^alpha * n1) + (k2 * sig(e2)^alpha * n2)]
	Eigen::Vector3d vel_ref = n1.cross(n2) - ((k1 * sig_e1_alpha * n1) + (k2 * sig_e2_alpha * n2));
	vel_ref = vel_ref.normalized() * 0.5;
	return vel_ref;
}

/* callbacks */
/**
 * @brief 负责轨迹跟踪
 * 
 * @param event 
 */
void track_timer_cb(const ros::TimerEvent& event) {
	if (!track_flag) return;
	// ROS_INFO("Timer callback triggered.");

	Eigen::Vector3d pos = Eigen::Vector3d::Zero();
	pos[0] = drone_state.position[0];
	pos[1] = drone_state.position[1];
	pos[2] = drone_state.position[2];
	Eigen::Vector3d vel = computeRefVel(pos);

	control_command.Command_ID += 1;
	control_command.source = "finite_time_tracking_node";
	control_command.Mode = prometheus_msgs::ControlCommand::Move;
	control_command.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
	control_command.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
	control_command.Reference_State.velocity_ref[0] = vel[0];
	control_command.Reference_State.velocity_ref[1] = vel[1];
	control_command.Reference_State.velocity_ref[2] = vel[2];
	control_command.Reference_State.Yaw_Rate_Mode = false; // True 代表偏航角速率控制
	control_command.Reference_State.yaw_ref = 0.0;

	control_pub.publish(control_command);
}

/**
 * @brief 触发轨迹跟踪
 * 
 * @param msg 
 */
void trigger_cb(const std_msgs::Bool::ConstPtr& msg) {
	ROS_INFO("Trigger callback triggered.");
	track_flag = !track_flag;
}

int main(int argc, char** argv) {
	/* ros node init */
	ros::init(argc, argv, "finite_time_tracking_node");
	ros::NodeHandle nh;
	ros::Rate rate(20);

	/* subscriber init */
	trigger_sub = nh.subscribe<std_msgs::Bool>(
		"/track_trigger",
		1,
		trigger_cb);
	state_sub = nh.subscribe<prometheus_msgs::DroneState>(
		"/prometheus/drone_state",
		10,
		[&](const prometheus_msgs::DroneState::ConstPtr& msg) {
			drone_state = *msg;
		});

	/* publisher init */
	control_pub = nh.advertise<prometheus_msgs::ControlCommand>(
		"/prometheus/control_command",
		10);
	refpath_pub = nh.advertise<visualization_msgs::Marker>(
		"/ref_path",
		10);
	vel_err_pub = nh.advertise<geometry_msgs::Vector3Stamped>(
		"/vel_err",
		10);

	/* timer init */
	std::cout << "Timer expected cycle time: " << rate.expectedCycleTime().toSec() << std::endl;
	track_timer = nh.createTimer(ros::Duration(rate.expectedCycleTime().toSec()), track_timer_cb);

	/* variable init */
	control_command.Command_ID = 0;
	control_command.source = "finite_time_tracking_node";
	control_command.Mode = prometheus_msgs::ControlCommand::Idle;
	control_command.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
	control_command.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
	control_command.Reference_State.velocity_ref[0] = 0.0;
	control_command.Reference_State.velocity_ref[1] = 0.0;
	control_command.Reference_State.velocity_ref[2] = 0.0;
	control_command.Reference_State.Yaw_Rate_Mode = false; // True 代表偏航角速率控制
	control_command.Reference_State.yaw_ref = 0.0;
	track_flag = false;
	computeRefPath(ref_path, 1.0, 5.0); // 计算参考轨迹存到ref_path
	std::cout << "Ref path points: " << ref_path.points.size() << std::endl;

	/* main loop */
	while (ros::ok()) {
		ros::spinOnce();

		refpath_pub.publish(ref_path);
		
		vel_err = geometry_msgs::Vector3Stamped();
		vel_err.header.stamp = ros::Time::now();
		vel_err.vector.x = control_command.Reference_State.velocity_ref[0] - drone_state.velocity[0];
		vel_err.vector.y = control_command.Reference_State.velocity_ref[1] - drone_state.velocity[1];
		vel_err.vector.z = control_command.Reference_State.velocity_ref[2] - drone_state.velocity[2];
		vel_err_pub.publish(vel_err);

		rate.sleep();
	}

	return 0;
}

/**
 * @brief 符号函数
 * 
 * @param x 
 * @return int 
 */
int sgn(const double x) {
	if (x > 0) {
		return 1;
	} else if (x < 0) {
		return -1;
	} else {
		return 0;
	}
}

/**
 * @brief 检查是否满足曲线方程
 * 
 * @param _pos 
 * @param _r 
 * @return true 
 * @return false 
 */
bool satisfiesEquations(const Eigen::Vector3d _pos, const double _r, const double _offset) {
	double x = _pos[0];
	double y = _pos[1];
	double z = _pos[2] - _offset;
	double phi1 = x*x + (z - 1.5)*(z - 1.5) - _r*_r;
	double phi2 = y*y + z*z - 4;

	return std::abs(phi1) < 5e-2 && std::abs(phi2) < 5e-2;
}