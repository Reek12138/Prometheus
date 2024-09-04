// **********************************************
// 实机集群定位转换节点
// **********************************************
//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

//topic 头文件
#include <prometheus_msgs/SwarmCommand_.h>
#include <prometheus_msgs/DroneState_.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

using namespace std;
#define NODE_NAME "swarm_offset"
#define MAX_NUM 40
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int swarm_num_uav;
ros::Publisher state_pub[MAX_NUM+1];
ros::Publisher path_pub[MAX_NUM+1];
ros::Publisher odom_pub[MAX_NUM+1];

ros::Subscriber state_listener[MAX_NUM+1];
ros::Subscriber path_listener[MAX_NUM+1];
ros::Subscriber odom_listener[MAX_NUM+1];

prometheus_msgs::DroneState_ swarm_state[MAX_NUM+1];
nav_msgs::Path drone_path[MAX_NUM+1];
nav_msgs::Odometry drone_odom[MAX_NUM+1];

vector<Eigen::Vector3f> offset_pos(4, Eigen::Vector3f::Zero());
vector<prometheus_msgs::DroneState_> Drone_state_;
vector<Eigen::Vector3f> drone_position_, drone_vel_, drone_position_offset_;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void drone_state_cb(const prometheus_msgs::DroneState_::ConstPtr& msg, int id);
void path_cb(const nav_msgs::Path::ConstPtr& msg, int id);

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg, int id);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//无人机局部坐标系相对于全局坐标系的偏移量
int main(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh("~");

    nh.param<int>("swarm_num_uav", swarm_num_uav, 2);

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
    
    // 初始化向量的大小
    Drone_state_.resize(swarm_num_uav);
    drone_position_.resize(swarm_num_uav, Eigen::Vector3f::Zero());
    drone_position_offset_.resize(swarm_num_uav, Eigen::Vector3f::Zero());
    drone_vel_.resize(swarm_num_uav, Eigen::Vector3f::Zero());
    

    for(int i = 1; i <= swarm_num_uav; i++){
        state_listener[i] = nh.subscribe<prometheus_msgs::DroneState_>("/uav" + std::to_string(i) + "/prometheus/drone_state", 10, boost::bind(drone_state_cb,_1,i-1));
        path_listener[i] = nh.subscribe<nav_msgs::Path>("/uav" + std::to_string(i) + "/prometheus/drone_trajectory", 10, boost::bind(path_cb,_1,i-1));
        odom_listener[i] = nh.subscribe<nav_msgs::Odometry>("/uav" + std::to_string(i) + "/prometheus/drone_odom", 10, boost::bind(odom_cb,_1,i-1));
        state_pub[i] = nh.advertise<prometheus_msgs::DroneState_>("/uav" + std::to_string(i) + "/prometheus/drone_state_exp", 1);
        path_pub[i] = nh.advertise<nav_msgs::Path>("/uav" + std::to_string(i) + "/prometheus/drone_trajectory_exp", 10);
        odom_pub[i] = nh.advertise<nav_msgs::Odometry>("/uav" + std::to_string(i) + "/prometheus/drone_odom_exp", 10);
        
    }

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

    // 让节点进入事件循环，保持运行
    while(ros::ok()){
    ros::spin();

    }

}

void drone_state_cb(const prometheus_msgs::DroneState_::ConstPtr& msg, int id){

    if (id < 0 || id >= Drone_state_.size()) {
        ROS_ERROR("Invalid drone ID: %d", id);
        return;
        }

    // Drone_state_[id] = *msg;
    drone_position_[id][0] = msg->position[0];
    drone_position_[id][1] = msg->position[1];
    drone_position_[id][2] = msg->position[2];

    drone_position_offset_[id][0] = drone_position_[id][0] + offset_pos[id][0];
    drone_position_offset_[id][1] = drone_position_[id][1] + offset_pos[id][1];
    drone_position_offset_[id][2] = drone_position_[id][2] + offset_pos[id][2];

    drone_vel_[id][0] = msg->velocity[0];
    drone_vel_[id][1] = msg->velocity[1];
    drone_vel_[id][2] = msg->velocity[2];

    swarm_state[id].header = msg->header;
    swarm_state[id].connected = msg->connected;
    swarm_state[id].armed = msg->armed;
    swarm_state[id].landed = msg->landed;
    swarm_state[id].mode = msg->mode;
    swarm_state[id].odom_valid = msg->odom_valid;

    swarm_state[id].position[0] = drone_position_offset_[id][0];
    swarm_state[id].position[1] = drone_position_offset_[id][1];
    swarm_state[id].position[2] = drone_position_offset_[id][2];
    
    swarm_state[id].rel_alt = msg->rel_alt;
    swarm_state[id].velocity = msg->velocity;
    swarm_state[id].attitude = msg->attitude;
    swarm_state[id].attitude_q = msg->attitude_q;
    swarm_state[id].attitude_rate = msg->attitude_rate;

    state_pub[(id+1)].publish(swarm_state[id]);
}

void path_cb(const nav_msgs::Path::ConstPtr& msg, int id) {
    if (id < 0 || id >= Drone_state_.size()) {
        ROS_ERROR("Invalid drone ID: %d", id);
        return;
    }

    if (msg->poses.empty()) {
        ROS_WARN("Received empty path for drone ID: %d", id);
        return;
    }

    drone_path[id].header = msg->header;

    // 确保 drone_path[id].poses 已分配足够空间
    drone_path[id].poses.resize(msg->poses.size());

    for (size_t i = 0; i < msg->poses.size(); ++i) {
        drone_path[id].poses[i].pose.position.x = msg->poses[i].pose.position.x + offset_pos[id][0];
        drone_path[id].poses[i].pose.position.y = msg->poses[i].pose.position.y + offset_pos[id][1];
        drone_path[id].poses[i].pose.position.z = msg->poses[i].pose.position.z + offset_pos[id][2];
        drone_path[id].poses[i].pose.orientation = msg->poses[i].pose.orientation;
    }

    path_pub[id+1].publish(drone_path[id]);
}



void odom_cb(const nav_msgs::Odometry::ConstPtr& msg, int id){
    if (id < 0 || id >= Drone_state_.size()) {
        ROS_ERROR("Invalid drone ID: %d", id);
        return;
    }
    // ROS_INFO("odom_cb called for drone ID: %d", id);
    
    drone_odom[id].header = msg->header;
    drone_odom[id].pose.pose.position.x = msg->pose.pose.position.x + offset_pos[id][0];
    drone_odom[id].pose.pose.position.y = msg->pose.pose.position.y + offset_pos[id][1];
    drone_odom[id].pose.pose.position.z = msg->pose.pose.position.z + offset_pos[id][2];

    drone_odom[id].pose.pose.orientation = msg->pose.pose.orientation;
    odom_pub[id+1].publish(drone_odom[id]);

    // ROS_INFO("Published modified odometry for drone ID: %d", id);
}
