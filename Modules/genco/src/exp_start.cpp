// **********************************************
// 实机集群起飞节点
// **********************************************

//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

//topic 头文件
#include <prometheus_msgs/SwarmCommand_.h>

#include <geometry_msgs/PoseStamped.h>


using namespace std;
#define NODE_NAME "exp_start"
#define MAX_NUM 40
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int swarm_num_uav;
ros::Publisher command_pub[MAX_NUM+1];


prometheus_msgs::SwarmCommand_ swarm_command[MAX_NUM+1];



vector<Eigen::Vector3f> init_pos(4, Eigen::Vector3f::Zero());
vector<Eigen::Vector3f> offset_pos(4, Eigen::Vector3f::Zero());




//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");

    nh.param<int>("swarm_num_uav", swarm_num_uav, 2);
    //无人机在全局坐标系下的初始位置
    nh.param<float>("init_pos_1_x", init_pos[0][0], 4.0);
    nh.param<float>("init_pos_1_y", init_pos[0][1], 4.0);
    nh.param<float>("init_pos_1_z", init_pos[0][2], 4.0);

    nh.param<float>("init_pos_2_x", init_pos[1][0], 4.0);
    nh.param<float>("init_pos_2_y", init_pos[1][1], -4.0);
    nh.param<float>("init_pos_2_z", init_pos[1][2], 8.0);

    nh.param<float>("init_pos_3_x", init_pos[2][0], -4.0);
    nh.param<float>("init_pos_3_y", init_pos[2][1], -4.0);
    nh.param<float>("init_pos_3_z", init_pos[2][2], 12.0);

    nh.param<float>("init_pos_4_x", init_pos[3][0], -4.0);
    nh.param<float>("init_pos_4_y", init_pos[3][1], 4.0);
    nh.param<float>("init_pos_4_z", init_pos[3][2], 16.0);

    //无人机局部坐标系相对于全局坐标系的偏移量
    nh.param<float>("offset_pos_1_x", offset_pos[0][0], 0.0);
    nh.param<float>("offset_pos_1_y", offset_pos[0][1], 0.0);
    nh.param<float>("offset_pos_1_z", offset_pos[0][2], 0.0);

    nh.param<float>("offset_pos_2_x", offset_pos[1][0], 0.0);
    nh.param<float>("offset_pos_2_y", offset_pos[1][1], 0.0);
    nh.param<float>("offset_pos_2_z", offset_pos[1][2], 0.0);

    nh.param<float>("offset_pos_3_x", offset_pos[2][0], 0.0);
    nh.param<float>("offset_pos_3_y", offset_pos[2][1], 0.0);
    nh.param<float>("offset_pos_3_z", offset_pos[2][2], 0.0);

    nh.param<float>("offset_pos_4_x", offset_pos[3][0], 0.0);
    nh.param<float>("offset_pos_4_y", offset_pos[3][1], 0.0);
    nh.param<float>("offset_pos_4_z", offset_pos[3][2], 0.0);

    
    

    for(int i = 1; i <= swarm_num_uav; i++){
        command_pub[i] = nh.advertise<prometheus_msgs::SwarmCommand_>("/uav" + std::to_string(i) + "/prometheus/swarm_command", 1);
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
    int start_flag = 0;
    while(start_flag == 0){
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Takeoff<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "请输入 1 把所有飞机移动至起始点 "<<endl;
        cin >> start_flag;
    }
    if(start_flag == 1){
        for(int i=1; i<=swarm_num_uav; i++){
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand_::Position_Control;
            swarm_command[i].source = "terminal_control";
            swarm_command[i].position_ref[0] = init_pos[i-1][0] - offset_pos[i-1][0];
            swarm_command[i].position_ref[1] = init_pos[i-1][1] - offset_pos[i-1][1];
            swarm_command[i].position_ref[2] = init_pos[i-1][2] - offset_pos[i-1][2];
            command_pub[i].publish(swarm_command[i]); //【发布】阵型
        }
    }
    while(ros::ok()){
        ros::spinOnce();
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "请输入 9 开始traj， 输入 0 hold"<<endl;
        cin >> start_flag;

        if(start_flag == 9 ){
            for(int i = 1; i <= swarm_num_uav; i++) 
            {
                swarm_command[i].header.stamp = ros::Time::now();
                swarm_command[i].header.frame_id = "world";
                swarm_command[i].source = "traj_start";
                swarm_command[i].Mode = prometheus_msgs::SwarmCommand_::Move;
                swarm_command[i].Move_mode = prometheus_msgs::SwarmCommand_::XYZ_VEL;
                swarm_command[i].velocity_ref = {0.0, 0.0, 0.0};
                swarm_command[i].position_ref = {0.0, 0.0, 0.0};
                swarm_command[i].acceleration_ref = {0.0, 0.0, 0.0};
                swarm_command[i].yaw_ref = 0.0;
                swarm_command[i].yaw_rate_ref = 0.0;
                command_pub[i].publish(swarm_command[i]);
            }

        }else if(start_flag == 0){
            for(int i=1; i<=swarm_num_uav; i++){
                swarm_command[i].Mode = prometheus_msgs::SwarmCommand_::Hold;
                swarm_command[i].source = "terminal_control";
                swarm_command[i].velocity_ref = {0.0, 0.0, 0.0};
                command_pub[i].publish(swarm_command[i]); //【发布】阵型
            }
        }else{
            cout << "输入错了"<<endl;
        }

        }
    
    }

