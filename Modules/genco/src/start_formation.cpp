// **********************************************
// 集群起飞节点
// **********************************************

//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

//topic 头文件
#include <prometheus_msgs/SwarmCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
#define NODE_NAME "start_formation"
#define MAX_NUM 40
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int swarm_num_uav;
bool sim_mode;
ros::Publisher command_pub[MAX_NUM+1];
ros::Subscriber state_listener[MAX_NUM+1];
prometheus_msgs::SwarmCommand swarm_command[MAX_NUM+1];
Eigen::Matrix<float, 4, 3> uav_goal_position;
Eigen::Matrix<float, 4, 3> uav_goal_velocity;
// Eigen::Vector3f drone_position_1, drone_position_2, drone_position_3, drone_position_4;
// Eigen::Vector3f drone_vel_1, drone_vel_2, drone_vel_3, drone_vel_4;
// prometheus_msgs::DroneState Drone_state_1_, Drone_state_2_, Drone_state_3_, Drone_state_4_;
// Eigen::Vector3f init_pos_1, init_pos_2, init_pos_3, init_pos_4;
vector<Eigen::Vector3f> init_pos(4, Eigen::Vector3f::Zero());
vector<prometheus_msgs::DroneState> Drone_state_;
vector<Eigen::Vector3f> drone_position_, drone_vel_;


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();
void pub_command();
void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg, int id);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");

    nh.param<int>("swarm_num_uav", swarm_num_uav, 4);
    nh.param<bool>("sim_mode",sim_mode,true);

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

    // 初始化向量的大小
    Drone_state_.resize(swarm_num_uav);
    drone_position_.resize(swarm_num_uav, Eigen::Vector3f::Zero());
    drone_vel_.resize(swarm_num_uav, Eigen::Vector3f::Zero());

    for(int i = 1; i <= swarm_num_uav; i++){
        command_pub[i] = nh.advertise<prometheus_msgs::SwarmCommand>("/uav" + std::to_string(i) + "/prometheus/swarm_command", 1);
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
    int start_flag = 0;

    printf_param();


    while(sim_mode && (start_flag == 0)){
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Start<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to disarm all the UAVs."<<endl;
        cin >> start_flag;

        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Idle;
            swarm_command[i].yaw_ref = 999;
            command_pub[i].publish(swarm_command[i]); //【发布】阵型
        }
    }
    start_flag = 0;
    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Takeoff<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please enter 1 to takeoff all the UAVs."<<endl;
        cin >> start_flag;

        for(int i = 1; i <= swarm_num_uav; i++) 
        {
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Takeoff;
            swarm_command[i].yaw_ref = 0.0;
            command_pub[i].publish(swarm_command[i]); //【发布】阵型
        }

        for(int i=1; i<=swarm_num_uav; i++){
            swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Position_Control;
            swarm_command[i].position_ref[0] = init_pos[i-1][0];
            swarm_command[i].position_ref[1] = init_pos[i-1][1];
            swarm_command[i].position_ref[2] = init_pos[i-1][2];
            command_pub[i].publish(swarm_command[i]); //【发布】阵型
        }
    }

    
    while(ros::ok()){
        ros::spinOnce();

        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Formation Flight Mission<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please choose the action: 9 for traj_genco Start, 0 for Move Position, 1 for Move Velocity, 2 for Hold, 3 for Land, 4 for Disarm..."<<endl;
        cin >> start_flag;
        if(start_flag == 0){
            cout << "输入无人机目标位置" << endl;
            for(int i=1; i<=swarm_num_uav; i++){
                cout << "无人机1的当前xyz位置： x: " << drone_position_[i-1][0] << " [m], y: " << drone_position_[i-1][1] << "[m], z: " <<drone_position_[i-1][2] << endl;
            }
            
            for (int i = 1; i <= swarm_num_uav; i++) {
                cout << "uav_" << i << " target_position x:--- x [m]" << endl;
                cin >> uav_goal_position(i-1,0);
                cout << "uav_" << i << " target_position y:--- y [m]" << endl;
                cin >> uav_goal_position(i-1,1);
                cout << "uav_" << i << " target_position z:--- z [m]" << endl;
                cin >> uav_goal_position(i-1,2);
            }

            for (int i = 1; i <= swarm_num_uav; i++) {
                swarm_command[i].header.stamp = ros::Time::now();
                swarm_command[i].header.frame_id = "world";
                swarm_command[i].source = "position_control";
                swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Move;
                swarm_command[i].Move_mode = prometheus_msgs::SwarmCommand::XYZ_POS;
                swarm_command[i].position_ref[0] = uav_goal_position(i-1, 0);
                swarm_command[i].position_ref[1] = uav_goal_position(i-1, 1);
                swarm_command[i].position_ref[2] = uav_goal_position(i-1, 2);
                swarm_command[i].velocity_ref = {0.0, 0.0, 0.0};
                swarm_command[i].acceleration_ref = {0.0, 0.0, 0.0};
                swarm_command[i].yaw_ref = 0.0;
                swarm_command[i].yaw_rate_ref = 0.0;
                command_pub[i].publish(swarm_command[i]);
            }
            
        }
        else if(start_flag == 1){
            cout << "输入无人机目标速度" << endl;
            for(int i=1; i<=swarm_num_uav; i++){
                cout << "无人机1的当前xyz速度： x: " << drone_vel_[i-1][0] << " [m], y: " << drone_vel_[i-1][1] << "[m], z: " <<drone_vel_[i-1][2] << endl;
            }
            
            for (int i = 1; i <= swarm_num_uav; i++) {
                cout << "uav_" << i << " target_velocity x:--- x [m/s]" << endl;
                cin >> uav_goal_velocity(i-1,0);
                cout << "uav_" << i << " target_velocity y:--- y [m/s]" << endl;
                cin >> uav_goal_velocity(i-1,1);
                cout << "uav_" << i << " target_velocity z:--- z [m/s]" << endl;
                cin >> uav_goal_velocity(i-1,2);
            }

            for (int i = 1; i <= swarm_num_uav; i++) {
                swarm_command[i].header.stamp = ros::Time::now();
                swarm_command[i].header.frame_id = "world";
                swarm_command[i].source = "velocity_control";
                swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Move;
                swarm_command[i].Move_mode = prometheus_msgs::SwarmCommand::XYZ_VEL;
                swarm_command[i].velocity_ref[0] = uav_goal_velocity(i-1, 0);
                swarm_command[i].velocity_ref[1] = uav_goal_velocity(i-1, 1);
                swarm_command[i].velocity_ref[2] = uav_goal_velocity(i-1, 2);
                swarm_command[i].position_ref = {0.0, 0.0, 0.0};
                swarm_command[i].acceleration_ref = {0.0, 0.0, 0.0};
                swarm_command[i].yaw_ref = 0.0;
                swarm_command[i].yaw_rate_ref = 0.0;
                command_pub[i].publish(swarm_command[i]);
            }
            
        }
        else if(start_flag == 2){
            for(int i = 1; i <= swarm_num_uav; i++) 
            {
                swarm_command[i].source = "terminal_control";
                swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Hold;
                command_pub[i].publish(swarm_command[i]); //【发布】阵型
            }
        }
        else if(start_flag == 3){
            for(int i = 1; i <= swarm_num_uav; i++) 
            {
                swarm_command[i].source = "terminal_control";
                swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Land;
                command_pub[i].publish(swarm_command[i]); //【发布】阵型
            }
        }
        else if (start_flag == 4)
        {
            for(int i = 1; i <= swarm_num_uav; i++) 
            {
                swarm_command[i].source = "terminal_control";
                swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Disarm;
                command_pub[i].publish(swarm_command[i]); //【发布】阵型
            }
        }
        else if (start_flag == 9)
        {
            for(int i = 1; i <= swarm_num_uav; i++) 
            {
                swarm_command[i].header.stamp = ros::Time::now();
                swarm_command[i].header.frame_id = "world";
                swarm_command[i].source = "traj_start";
                swarm_command[i].Mode = prometheus_msgs::SwarmCommand::Move;
                swarm_command[i].Move_mode = prometheus_msgs::SwarmCommand::XYZ_VEL;
                swarm_command[i].velocity_ref = {0.0, 0.0, 0.0};
                swarm_command[i].position_ref = {0.0, 0.0, 0.0};
                swarm_command[i].acceleration_ref = {0.0, 0.0, 0.0};
                swarm_command[i].yaw_ref = 0.0;
                swarm_command[i].yaw_rate_ref = 0.0;
                command_pub[i].publish(swarm_command[i]);
            }
        }
        else
        {
            cout << "Wrong input."<<endl;
        }
        ros::Duration(1.0).sleep();
    }
    return 0;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Formation Flight Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "swarm_num_uav   : "<< swarm_num_uav <<endl;
    
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg, int id){

    if (id < 0 || id >= Drone_state_.size()) {
        ROS_ERROR("Invalid drone ID: %d", id);
        return;
        }

    // Drone_state_[id] = *msg;
    drone_position_[id][0] = msg->position[0];
    drone_position_[id][1] = msg->position[1];
    drone_position_[id][2] = msg->position[2];

    drone_vel_[id][0] = msg->velocity[0];
    drone_vel_[id][1] = msg->velocity[1];
    drone_vel_[id][2] = msg->velocity[2];

}