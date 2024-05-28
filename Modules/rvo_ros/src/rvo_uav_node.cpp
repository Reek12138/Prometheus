/**
 * @file rvo_uav_node.cpp
 * @author Jialong Zeng
 * @brief 利用 RVO2 库实现 2 架无人机互换位置过程中避碰
 * 				参考 https://gamma.cs.unc.edu/RVO2/documentation/2.0/using.html
 * 				- 输入：无人机状态信息(/uav?/prometheus/drone_state_manager)
 * 				- 输出：无人机控制指令(/uav?/prometheus/swarm_command_manager)
 * 				基本测试流程：
 * 				1. 启动集群仿真环境 roslaunch swarm_basic sitl_swarm_manager.launch
 * 				2. 打开QGC和Rviz可视化
 * 				3. 启动RVO控制节点 rosrun rvo_ros rvo_uav_node
 * 				4. QGC内解锁两台无人机
 * 				5. 发布开关话题 rosotopic pub -1 /flag_rvo std_msgs/Bool "data: false"
 * 
 * @version 0.1
 * @date 2024-05-27
 * @note 1. 无人机数量只能是2
 * 			 2. 勉强有效果，但是快到半径处的时候会有震荡，可能需要调参或者改进代码逻辑，还没有具体测试原因
 * 			 3. 还没有实机测试
 * 			 4. 只支持单次互换，重复互换需要重新启动节点，需要改进代码
 * 			 5. 还没有测试控制频率、RVO运行频率对效果的影响
 * 
 */

/* sys include */
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Bool.h>

/* external pkg */
#include "../rvo_lib/RVO.h" // RVO2 库头文件
#include "prometheus_msgs/DroneState.h"
#include "prometheus_msgs/SwarmCommand.h"

/* variable */
bool flag_rvo; // 用于判断是否启动 RVO 控制，人为开关
int num_uav; // 集群内无人机数量，默认是 2，目前也只支持 2 架无人机
std::vector<ros::Subscriber> state_subs; // 订阅无人机状态
std::vector<ros::Publisher> control_pubs; // 发布无人机控制指令
std::vector<prometheus_msgs::DroneState> drone_states; // 存储所有无人机状态信息
std::vector<prometheus_msgs::SwarmCommand> swarm_commands; // 存储所有要发布的无人机控制指令(暂时没用到)
std::vector<RVO::Vector2> goals; // 存储 RVO 内智能体的目标位置
ros::Timer rvo_timer;	// 定时器，定期运行 RVO
RVO::RVOSimulator* sim; // RVO 模拟器
ros::Subscriber flag_rvo_sub; // 订阅人为 RVO 控制开关

/* callbacks */
/**
 * @brief RVO 控制回调函数，先将无人机状态信息同步到 RVO 模拟器中，然后运行 RVO 模拟器，最后将 RVO 模拟器输出的期望速度命令发送给无人机
 * 
 * @param event 
 */
void rvo_timer_cb(const ros::TimerEvent& event) {
	if (!flag_rvo) return;
	// 将RVO中的agent和无人机状态同步
	for (int i = 0; i < num_uav; i++) {
		float x = drone_states[i].position[0];
		float y = drone_states[i].position[1];
		sim->setAgentPosition(i, RVO::Vector2(x, y));
		x = drone_states[i].velocity[0];
		y = drone_states[i].velocity[1];
		sim->setAgentVelocity(i, RVO::Vector2(x, y));
		if (RVO::absSq(goals[i] - sim->getAgentPosition(i)) < sim->getAgentRadius(i)) {
			sim->setAgentPrefVelocity(i, RVO::Vector2(0, 0));
		}
		else {
			sim->setAgentPrefVelocity(i, RVO::normalize(goals[i] - sim->getAgentPosition(i)));
		}
	}
	// 运行RVO模拟器
	sim->doStep();

	// 将RVO输出的速度命令发送给无人机
	for (int i = 0; i < num_uav; i++) {
		RVO::Vector2 cmd_vel = sim->getAgentVelocity(i);
		prometheus_msgs::SwarmCommand cmd = prometheus_msgs::SwarmCommand();
		cmd.Command_ID += 1;
		cmd.Mode = prometheus_msgs::SwarmCommand::Move;
		cmd.Move_mode = prometheus_msgs::SwarmCommand::XY_VEL_Z_POS;
		cmd.position_ref[0] = 0;
		cmd.position_ref[1] = 0;
		cmd.position_ref[2] = 2;
		cmd.velocity_ref[0] = cmd_vel.x();
		cmd.velocity_ref[1] = cmd_vel.y();
		cmd.velocity_ref[2] = 0;
		cmd.acceleration_ref[0] = 0;
		cmd.acceleration_ref[1] = 0;
		cmd.acceleration_ref[2] = 0;
		cmd.yaw_ref = 0;
		cmd.yaw_rate_ref = 0;
		control_pubs[i].publish(cmd);
	}

	// 输出调试信息
	for (int i = 0; i < num_uav; i++) {
		std::cout << "uav" << i+1 << ": " << std::endl;
		std::cout << "  pos: " << sim->getAgentPosition(i).x() << ", " << sim->getAgentPosition(i).y() << std::endl;
		std::cout << "  goal: " << goals[i].x() << ", " << goals[i].y() << std::endl;
		std::cout << "  pref_vel: " << sim->getAgentPrefVelocity(i).x() << ", " << sim->getAgentPrefVelocity(i).y() << std::endl;
		std::cout << "  cmd_vel: " << sim->getAgentVelocity(i).x() << ", " << sim->getAgentVelocity(i).y() << std::endl;
	}
}

/**
 * @brief 用于人为开关 RVO 控制，初次开启时，将无人机当前位置作为其他无人机的目标位置，以此达到一个目标互换的效果
 * @warning 仅支持单次互换，第二次开启不知道会有什么效果
 * 
 * @param msg 
 */
void flag_rvo_cb(const std_msgs::Bool::ConstPtr& msg) {
	// if flag_rvo from false to true, then set the current pos of uavs as goals
	if (!flag_rvo) {
		for (int i = 0; i < num_uav; i++) {
			float x = drone_states[i].position[0];
			float y = drone_states[i].position[1];
			sim->addAgent(RVO::Vector2(x, y));

			x = drone_states[num_uav - i - 1].position[0];
			y = drone_states[num_uav - i - 1].position[1];
			goals.push_back(RVO::Vector2(x, y));
		}
		std::cout << "RVO start." << std::endl;
	}
	flag_rvo = !flag_rvo;
}

/**
 * @brief 订阅无人机状态信息并存储起来
 * 
 * @param msg 
 * @param id 
 */
void state_cb(const prometheus_msgs::DroneState::ConstPtr &msg, int id) {
	drone_states[id] = *msg;
}

int main(int argc, char** argv) {
	/* ros node init */
	ros::init(argc, argv, "rvo_uav_node");
	ros::NodeHandle nh;
	ros::Rate rate(20);

	/* variable init */
	nh.param("num_uav", num_uav, 2);
	flag_rvo = false;
	// RVO simulator 基本初始化
	sim = new RVO::RVOSimulator(); // craete a new simulator instance
	sim->setAgentDefaults(3.0f, 5, 10.0f, 10.0f, 1.2f, 0.3f); // set the default parameters of the agents
	sim->setTimeStep(0.25f); // set the time step of the simulation
	// 初始化无人机状态信息和控制指令
	for (int i = 0; i < num_uav; i++) {
		prometheus_msgs::SwarmCommand cmd = prometheus_msgs::SwarmCommand();
		cmd.Command_ID = 0;
		cmd.source = "rvo_uav_node";
		cmd.Mode = prometheus_msgs::SwarmCommand::Idle;
		cmd.Move_mode = prometheus_msgs::SwarmCommand::XY_VEL_Z_POS;
		cmd.position_ref[0] = 0;
		cmd.position_ref[1] = 0;
		cmd.position_ref[2] = 0;
		cmd.velocity_ref[0] = 0;
		cmd.velocity_ref[1] = 0;
		cmd.velocity_ref[2] = 0;
		cmd.acceleration_ref[0] = 0;
		cmd.acceleration_ref[1] = 0;
		cmd.acceleration_ref[2] = 0;
		cmd.yaw_ref = 0;
		cmd.yaw_rate_ref = 0;
		swarm_commands.push_back(cmd);
	}
	for (int i = 0; i < num_uav; i++) {
		prometheus_msgs::DroneState state = prometheus_msgs::DroneState();
		state.connected = false;
		drone_states.push_back(state);
	}

	/* subscriber init */
	flag_rvo_sub = nh.subscribe<std_msgs::Bool>(
		"/flag_rvo",
		1,
		flag_rvo_cb);
	for (int i = 0; i < num_uav; i++) {
		ros::Subscriber sub = nh.subscribe<prometheus_msgs::DroneState>(
			"/uav" + std::to_string(i+1) + "/prometheus/drone_state_manager",
			1,
			boost::bind(state_cb, _1, i));
		state_subs.push_back(sub);
	}

	/* publisher init */
	for (int i = 0; i < num_uav; i++) {
		ros::Publisher pub = nh.advertise<prometheus_msgs::SwarmCommand>(
			"/uav" + std::to_string(i+1) + "/prometheus/swarm_command_manager",
			1);
		control_pubs.push_back(pub);
	}

	/* timer init */
	rvo_timer = nh.createTimer(
		ros::Duration(rate.expectedCycleTime().toSec()),
		rvo_timer_cb);

	std::cout << "RVO node start." << std::endl;

	/* main loop */
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}

	delete sim;
	return 0;
}
