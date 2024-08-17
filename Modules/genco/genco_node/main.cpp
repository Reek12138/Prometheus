#include <iostream>
#include <Eigen/Core>
#include "genco_node.h"
#include <ros/ros.h>
//topic 头文件
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <geometry_msgs/PoseStamped.h>
#include <algorithm>
#include <vector>
#include <tuple>

#include <nlopt.hpp>


#define NODE_NAME "genco"
#define MAX_NUM 40
using namespace std;

ros::Publisher command_pub;
ros::Subscriber state_listener[MAX_NUM+1];
ros::Subscriber command_listener;
prometheus_msgs::SwarmCommand swarm_command;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
vector<Eigen::Vector3f> drone_position_, drone_vel_;//uav1->drone_postion_[0], drone_vel_[0]
vector<prometheus_msgs::DroneState> Drone_state_;//uav1->Drone_state_[0]
prometheus_msgs::SwarmCommand Drone_command_;
int swarm_num_uav;
Eigen::Vector3f init_pos, end_pos;//uav1->init_pos[0]
bool sim_mode;
bool start_flag, traj_flag;//开始标志
int self_id;
std::vector<TrajectoryPoint> trajectory;

bool genco_flag[MAX_NUM+1];//切入genco标志位

int collision_num[MAX_NUM+1];//碰撞判断计数

int non_collision_num[MAX_NUM+1];//碰撞判断计数

float collision_therahold;//碰撞边界
float safe_therahold;//安全边界
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg, int id);
void printf_param();
void drone_command_cb(const prometheus_msgs::SwarmCommand::ConstPtr& msg);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");

    nh.param<int>("swarm_num_uav", swarm_num_uav, 4);
    nh.param<int>("self_id", self_id, 0);

    nh.param<float>("init_pos_x", init_pos[0], 4.0);
    nh.param<float>("init_pos_y", init_pos[1], 4.0);
    nh.param<float>("init_pos_z", init_pos[2], 4.0);

    nh.param<float>("end_pos_x", end_pos[0], 4.0);
    nh.param<float>("end_pos_y", end_pos[1], -4.0);
    nh.param<float>("end_pos_z", end_pos[2], 8.0);

    nh.param<float>("collision_therahold", collision_therahold, 2.0);
    nh.param<float>("safe_therahold", safe_therahold, 2.0);

    

    // 初始化向量的大小
    Drone_state_.resize(swarm_num_uav);
    drone_position_.resize(swarm_num_uav, Eigen::Vector3f::Zero());
    drone_vel_.resize(swarm_num_uav, Eigen::Vector3f::Zero());
    std::fill(genco_flag, genco_flag + MAX_NUM+1, false);
    std::fill(collision_num, collision_num + MAX_NUM+1, 0);
    std::fill(non_collision_num, non_collision_num + MAX_NUM+1, 0);

    
    command_pub = nh.advertise<prometheus_msgs::SwarmCommand>("/uav" + std::to_string(self_id) + "/prometheus/swarm_command", 1);

    command_listener = nh.subscribe<prometheus_msgs::SwarmCommand>("/uav" + std::to_string(self_id) + "/prometheus/swarm_command", 1, drone_command_cb);

    for(int i = 1; i <= swarm_num_uav; i++){
        state_listener[i] = nh.subscribe<prometheus_msgs::DroneState>("/uav" + std::to_string(i) + "/prometheus/drone_state", 10, boost::bind(drone_state_cb,_1,i-1));
    }

    // ros::Duration(1.0).sleep();
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // Waiting for input
    start_flag = false;
    traj_flag = false;

    // 定义时间间隔
    std::vector<float> time_intervals = {0.0f, 2.5f, 5.0f, 7.5f, 10.0f, 12.5f, 15.0f, 17.5f, 20.0f};
    float total_time = 20.0f;

    // 定义无人机的起始和结束位置
    // std::vector<Eigen::Vector3f> start_point = {
    //     Eigen::Vector3f(4.0f, 4.0f, 4.0f),
    //     Eigen::Vector3f(4.0f, -4.0f, 6.0f),
    //     Eigen::Vector3f(-4.0f, -4.0f, 8.0f),
    //     Eigen::Vector3f(-4.0f, 4.0f, 10.0f)
    // };

    // std::vector<Eigen::Vector3f> end_point = {
    //     Eigen::Vector3f(-4.0f, -4.0f, 10.0f),
    //     Eigen::Vector3f(-4.0f, 4.0f, 8.0f),
    //     Eigen::Vector3f(4.0f, 4.0f, 6.0f),
    //     Eigen::Vector3f(4.0f, -4.0f, 4.0f)
    // };

    Eigen::Vector3f start_point(init_pos[0], init_pos[1], init_pos[2]);
    Eigen::Vector3f end_point(end_pos[0], end_pos[1], end_pos[2]);
    // 生成轨迹
    trajectory = generateTrajectory(start_point, end_point, total_time, time_intervals);

    printf_param();

    nlopt::opt opt(nlopt::GN_CRS2_LM, 3);


    //算法主要逻辑
    while(ros::ok()){
        ros::spinOnce();

        if(Drone_command_.source == "traj_start"){
            start_flag = true;
            traj_flag = true;

            swarm_command.header.stamp = ros::Time::now();
            swarm_command.header.frame_id = "world";
            swarm_command.source = "traj_control";
            swarm_command.Mode = prometheus_msgs::SwarmCommand::Move;
            swarm_command.Move_mode = prometheus_msgs::SwarmCommand::XYZ_VEL;
            swarm_command.velocity_ref[0] = trajectory[0].velocity[0];
            swarm_command.velocity_ref[1] = trajectory[0].velocity[1];
            swarm_command.velocity_ref[2] = trajectory[0].velocity[2];
            swarm_command.position_ref = {0.0, 0.0, 0.0};
            swarm_command.acceleration_ref = {0.0, 0.0, 0.0};
            swarm_command.yaw_ref = 0.0;
            swarm_command.yaw_rate_ref = 0.0;
            command_pub.publish(swarm_command);
        }else{
            // start_flag = false;
            // traj_flag = false;
        }
        
        for(int i=0; i<swarm_num_uav; i++){
            if(start_flag == true && genco_flag[i] == true){
                Eigen::Vector3f Vel_, Pos_;
                float therahold_slope;
                std::tie(Pos_, Vel_, therahold_slope) = RelativeVelocity2NewFrame(drone_position_[self_id-1], drone_vel_[self_id-1], drone_position_[i], drone_vel_[i], safe_therahold);

                float slope = calculateSlope(Vel_);
                cout << "uav"<<to_string(self_id)<<"可能与uav" << std::to_string(i+1) << "发生冲突" << endl;
                cout << "uav" << to_string(i+1) << " : x= " << Pos_[0]<<" m, y= "<<Pos_[1]<<" m, z= "<<Pos_[2]<<" m"<<endl;
                
                if(slope > therahold_slope){


                }
            }
        }




        ros::Duration(0.5).sleep();
    }
    return 0;
}

//这里uav1 id=0
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg, int id){

    if (id < 0 || id >= Drone_state_.size()) {
        ROS_ERROR("Invalid drone ID: %d", id);
        return;
        }

    Drone_state_[id] = *msg;
    drone_position_[id][0] = msg->position[0];
    drone_position_[id][1] = msg->position[1];
    drone_position_[id][2] = msg->position[2];

    drone_vel_[id][0] = msg->velocity[0];
    drone_vel_[id][1] = msg->velocity[1];
    drone_vel_[id][2] = msg->velocity[2];

    //判断是否进入碰撞范围
    if(id != self_id-1){
        float distance = (drone_position_[id] - drone_position_[self_id-1]).norm();
        if(distance <= collision_therahold){
            collision_num[id]++;
            if(collision_num[id] >= 5){
                genco_flag[id] = true;
            }
        }
        else{
            non_collision_num[id]++;
            if(non_collision_num[id] >= 5){
                genco_flag[id] = false;
            }
        }
    }else{
        genco_flag[id] = false;
    }

}
void drone_command_cb(const prometheus_msgs::SwarmCommand::ConstPtr& msg){
    Drone_command_ = *msg;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Formation Flight Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "swarm_num_uav   : "<< swarm_num_uav <<endl;
    cout << "UAV_ID :  " << self_id <<endl;
    // cout << "Drone_state size : "<< Drone_state_.size() <<endl;
    // 输出轨迹信息
    printTrajectory(trajectory, "UAV" + std::to_string(self_id));

    
}

