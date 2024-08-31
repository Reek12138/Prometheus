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
#include <random>

#include <nlopt.hpp>


#define NODE_NAME "genco"
#define MAX_NUM 40
#define PI 3.1415926
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

bool collision_flag[MAX_NUM+1];//切入genco标志位
bool genco_flag[MAX_NUM+1];//切入genco标志位
bool self_genco_flag;

int collision_num[MAX_NUM+1];//碰撞判断计数

int non_collision_num[MAX_NUM+1];//碰撞判断计数

float collision_therahold;//碰撞边界
float safe_therahold;//安全边界

float total_time ;
bool nlopt_flag;

float ros_t;

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

    nh.param<float>("total_time", total_time, 80.0);
    nh.param<float>("ros_t", ros_t, 1.0);

    

    // 初始化向量的大小
    Drone_state_.resize(swarm_num_uav);
    drone_position_.resize(swarm_num_uav, Eigen::Vector3f::Zero());
    drone_vel_.resize(swarm_num_uav, Eigen::Vector3f::Zero());
    std::fill(collision_flag, collision_flag + MAX_NUM+1, false);
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

    

    Data data(swarm_num_uav);

    float last_psi = 0.0, last_theta = 0.0;

    //算法主要逻辑
    while(ros::ok()){
        ros::spinOnce();

        if(Drone_command_.source == "traj_start"){
            start_flag = true;
            traj_flag = true;
            self_genco_flag = false;
        }else{
            // start_flag = false;
            // traj_flag = false;
        }

        
        if(start_flag == true){

            Eigen::Vector3f target_dis_now = end_point - drone_position_[self_id-1];
            auto[target_psi, target_theta] = Velocity2Angles(target_dis_now);
            // std::cout << "目标psi： "<< target_psi << " 目标theta： " << target_theta  << std::endl;

            nlopt_flag = true;

                data.psi_g = target_psi;
                data.theta_g = target_theta;
                data.psi_min = -PI;
                data.psi_max = PI;
                data.theta_min = -PI/2;
                data.theta_max = PI/2;
                data.V = trajectory[0].velocity.norm();

                float minf_now = std::numeric_limits<double>::infinity();
                float genco_psi = 0.0;
                float genco_theta = 0.0;

                for(int i=0; i<100; i++){
                    for(int j=0; j<50; j++){
                        float psi = -PI + ((2*PI)/100)*i;
                        float theta = -PI/2 +((PI)/50)*j;
                        bool constrain_flag = true;

                        for(int k = 0; k<swarm_num_uav; k++){
                            if(genco_flag[k] == true){
                                Eigen::Vector3f v = Angles2Velocity(psi, theta, data.V);
                                Eigen::Vector3f v_ = data.R[k] * v;
                                auto [psi_i_, theta_i_] = Velocity2Angles(v_);
                                constrain_flag = (cos(psi_i_) * cos(theta_i_) - sin(theta_i_) / data.therahold_slope - data.target_xy_[k][1]) > 0;
                            }
                        }


                        if(constrain_flag == true){
                            // float obj_value = std::pow((psi - target_psi), 2) + std::pow((theta - target_theta), 2) + std::pow((psi - last_psi), 2) + std::pow((theta - last_theta), 2);
                            float obj_value = std::pow((psi - target_psi), 2) + std::pow((theta - target_theta), 2);

                            if(obj_value < minf_now){
                                genco_psi = psi;
                                genco_theta = theta;

                                minf_now = obj_value;
                            }
                        }
                        

                    }
                }

                    last_psi = genco_psi;
                    last_theta = genco_theta;
                    std::cout << "目标psi: " << target_psi<<" 目标theta: " << target_theta<< std::endl;
                    std::cout << "当前psi: " << genco_psi<<" 当前theta: " << genco_theta<< "当前f min=" <<minf_now << std::endl;
                    Eigen::Vector3f genco_V = Angles2Velocity(genco_psi, genco_theta, trajectory[0].velocity.norm());
                    std::cout <<"无人机 " << self_id<< "  genco速度 xyz: X = " << genco_V.x() << " Y = " << genco_V.y() << " Z = " << genco_V.z() << std::endl;
                    std::cout << "--------------------------------------------------------------------------------" << std::endl;
                    swarm_command.header.stamp = ros::Time::now();
                    swarm_command.header.frame_id = "world";
                    swarm_command.source = "genco_control";
                    swarm_command.Mode = prometheus_msgs::SwarmCommand::Move;
                    swarm_command.Move_mode = prometheus_msgs::SwarmCommand::XYZ_VEL;
                    swarm_command.velocity_ref[0] = genco_V.x();
                    swarm_command.velocity_ref[1] = genco_V.y();
                    swarm_command.velocity_ref[2] = genco_V.z();
                    swarm_command.position_ref = {0.0, 0.0, 0.0};
                    swarm_command.acceleration_ref = {0.0, 0.0, 0.0};
                    swarm_command.yaw_ref = 0.0;
                    swarm_command.yaw_rate_ref = 0.0;
                    command_pub.publish(swarm_command);
            
            
        }
        
        
        for(int i=0; i<swarm_num_uav; i++){

                if(collision_flag[i] == true){
                    Eigen::Vector3f Vel_, Pos_;
                    float therahold_slope;
                    Eigen::Matrix3f R;
                    std::tie(Pos_, Vel_, therahold_slope, R) = RelativeVelocity2NewFrame(drone_position_[self_id-1], drone_vel_[self_id-1], drone_position_[i], drone_vel_[i], safe_therahold);

                    float slope = calculateSlope(Vel_);

                    if(Pos_.norm() <= safe_therahold){
                        cout << "###########与uav " << std::to_string(i+1) << " 的距离 ：" << Pos_.norm() <<" (m)" << std::endl;
                    }

                     // 在碰撞锥内有碰撞风险
                    if(slope > therahold_slope){
                        cout << "uav"<<to_string(self_id)<<"可能与uav" << std::to_string(i+1) << "发生冲突" << endl;
                        // cout << "uav" << to_string(i+1) << " : x= " << Pos_[0]<<" m, y= "<<Pos_[1]<<" m, z= "<<Pos_[2]<<" m"<<endl;
                        self_genco_flag = true;
                        genco_flag[i] = true;
                        traj_flag = false;
                        // 变换后坐标系内本方无人机i和对方无人机j的速度，角度
                        Eigen::Vector3f v_i_ = R * drone_vel_[self_id-1];
                        Eigen::Vector3f v_j_ = R * drone_vel_[i];

                        // "偏航角 (ψ): " << psi   "俯仰角 (θ): " << theta
                        auto[psi_i_, theta_i_] = Velocity2Angles(v_i_);
                        auto[psi_j_, theta_j_] = Velocity2Angles(v_j_);

                        float X_ = cos(psi_j_)*cos(theta_j_) - sin(theta_j_ )/ therahold_slope;
                        float Y_ = cos(psi_i_)*cos(theta_i_) - sin(theta_i_ )/ therahold_slope;
                        float K_ = v_j_.norm() / v_i_.norm();

                        Eigen::Vector2f target_xy_ = calculatePerpendicularIntersection(X_, Y_, K_);

                        data.therahold_slope = therahold_slope;
                        data.target_xy_[i] = target_xy_;
                        data.R[i] = R;

                    }else{
                    genco_flag[i] = false;
                    }
                    // 检查是否所有元素都是 false
                    for (size_t i = 0; i < sizeof(genco_flag)/sizeof(genco_flag[0]); ++i) {
                        if (!genco_flag[i]) {
                            self_genco_flag = false;
                            break;
                        }
                    }
            }
        }




        ros::Duration(ros_t).sleep();
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
                collision_flag[id] = true;
            }
        }
        else{
            non_collision_num[id]++;
            if(non_collision_num[id] >= 5){
                collision_flag[id] = false;
            }
        }
    }else{
        collision_flag[id] = false;
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
    cout << "ros_t :" << ros_t << endl;
    // cout << "Drone_state size : "<< Drone_state_.size() <<endl;
    // 输出轨迹信息
    printTrajectory(trajectory, "UAV" + std::to_string(self_id));

    
}

// // 目标函数
// double objective_function(const std::vector<double> &x, std::vector<double> &grad, void *data) {
//     Data* d = static_cast<Data*>(data);
//     double psi_i = x[0];
//     double theta_i = x[1];

//     // 计算目标函数值
//     double obj_value;
//     obj_value = std::pow((psi_i - d->psi_g), 2) + std::pow((theta_i - d->theta_g), 2);
   

//     // 如果需要计算梯度
//     if (!grad.empty()) {
//         grad[0] = 2 * (psi_i - d->psi_g);
//         grad[1] = 2 * (theta_i - d->theta_g);
//     }

//     return obj_value;
// }

// double objective_function2(const std::vector<double> &x, std::vector<double> &grad, void *data) {
//     Data* d = static_cast<Data*>(data);
//     double psi_i = x[0];
//     double theta_i = x[1];
//     // if(!grad.empty()){
//     //     grad[0] = 0;
//     //     grad[1] = 0;
//     // }

//     // 计算目标函数值
//     double obj_value;
//         Eigen::Vector3f v = Angles2Velocity(psi_i, theta_i, d->V);

//         for(int i=0; i<swarm_num_uav; i++){
//             if(i != (self_id-1) && genco_flag[i] == true){

//                 Eigen::Vector3f v_ = d->R[i] * v;
//                 auto [psi_i_, theta_i_] = Velocity2Angles(v_);

//                 obj_value += std::pow((cos(psi_i_) * cos(theta_i_) - sin(theta_i_) / d->therahold_slope) - (d->target_xy_[i][1]),2);

//                 // if(!grad.empty()){
//                 //     // 计算约束函数的梯度
//                 //     double dg_dpsi_i_ = -std::sin(psi_i_) * std::cos(theta_i_);
//                 //     double dg_dtheta_i_ = -std::cos(psi_i_) * std::sin(theta_i_) - std::cos(theta_i_) / d->therahold_slope;
//                 //     grad[0] += dg_dpsi_i_ * d->R[i](0, 0) + dg_dtheta_i_ * d->R[i](1, 0);
//                 //     grad[1] += dg_dpsi_i_ * d->R[i](0, 1) + dg_dtheta_i_ * d->R[i](1, 1);
//                 // }
//             }
//         }
//         return obj_value;
// }



// // 约束函数
// // void constraint_function(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data) {
// //     Data* d = static_cast<Data*>(data);
// //     double psi_i = x[0];
// //     double theta_i = x[1];

// //     // 约束1: 检查 psi_i 和 theta_i 是否在 [psi_min, psi_max] 和 [theta_min, theta_max] 范围内
// //     result[0] = psi_i - d->psi_min;  // psi_i >= psi_min
// //     result[1] = d->psi_max - psi_i;  // psi_i <= psi_max
// //     result[2] = theta_i - d->theta_min;  // theta_i >= theta_min
// //     result[3] = d->theta_max - theta_i;  // theta_i <= theta_max

// //     // 初始化梯度数组
// //     if (grad) {
// //         std::fill(grad, grad + n, 0.0);
// //         grad[0] = 1.0;  // 对应 result[0] 的 psi_i 偏导数
// //         grad[1] = -1.0; // 对应 result[1] 的 psi_i 偏导数
// //         grad[2] = 1.0;  // 对应 result[2] 的 theta_i 偏导数
// //         grad[3] = -1.0; // 对应 result[3] 的 theta_i 偏导数
// //     }

    
// //          Eigen::Vector3f v = Angles2Velocity(psi_i, theta_i, d->V);

// //         for (int i = 0; i < swarm_num_uav; i++) {
// //             if (i != (self_id - 1) && genco_flag[i] == true) {

// //                 Eigen::Vector3f v_ = d->R[i] * v;
// //                 auto [psi_i_, theta_i_] = Velocity2Angles(v_);

// //                 result[4 + i] = cos(psi_i_) * cos(theta_i_) - sin(theta_i_) / d->therahold_slope - d->target_xy_[i][1];

// //                 if (grad) {
// //                     // 计算约束函数的梯度
// //                     double dg_dpsi_i_ = -std::sin(psi_i_) * std::cos(theta_i_);
// //                     double dg_dtheta_i_ = -std::cos(psi_i_) * std::sin(theta_i_) - std::cos(theta_i_) / d->therahold_slope;

// //                     // 填充对 psi_i 和 theta_i 的梯度
// //                     grad[4 + i] += dg_dpsi_i_ * d->R[i](0, 0) + dg_dtheta_i_ * d->R[i](1, 0);
// //                     grad[4 + i + swarm_num_uav] += dg_dpsi_i_ * d->R[i](0, 1) + dg_dtheta_i_ * d->R[i](1, 1);
// //                 }

// //                 // std::cout << "增加了第 " << i << " 个genco约束 " << std::endl;
// //             } else {
// //                 result[4 + i] = 0;
// //             }
// //         }
    
   
// // }

// void constraint_function(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data) {
//     Data* d = static_cast<Data*>(data);
//     double psi_i = x[0];
//     double theta_i = x[1];

//     // 约束1: 检查 psi_i 和 theta_i 是否在 [psi_min, psi_max] 和 [theta_min, theta_max] 范围内
//     result[0] = psi_i - d->psi_min;  // psi_i >= psi_min
//     result[1] = d->psi_max - psi_i;  // psi_i <= psi_max
//     result[2] = theta_i - d->theta_min;  // theta_i >= theta_min
//     result[3] = d->theta_max - theta_i;  // theta_i <= theta_max

//     // if (grad) {
//     //     // 初始化梯度数组
//     //     std::fill(grad, grad + n * m, 0.0);  // 清空整个梯度数组

//     //     grad[0 * n + 0] = 1.0;  // 对应 result[0] 的 psi_i 偏导数
//     //     grad[1 * n + 0] = -1.0; // 对应 result[1] 的 psi_i 偏导数
//     //     grad[2 * n + 1] = 1.0;  // 对应 result[2] 的 theta_i 偏导数
//     //     grad[3 * n + 1] = -1.0; // 对应 result[3] 的 theta_i 偏导数
//     // }

//     // 处理 swarm_num_uav 的额外约束
//     Eigen::Vector3f v = Angles2Velocity(psi_i, theta_i, d->V);

//     for (int i = 0; i < swarm_num_uav; i++) {
//         if (i != (self_id - 1) && genco_flag[i] == true) {
//             Eigen::Vector3f v_ = d->R[i] * v;
//             auto [psi_i_, theta_i_] = Velocity2Angles(v_);

//             result[4 + i] = cos(psi_i_) * cos(theta_i_) - sin(theta_i_) / d->therahold_slope - d->target_xy_[i][1];

//             // if (grad) {
//             //     double dg_dpsi_i_ = -std::sin(psi_i_) * std::cos(theta_i_);
//             //     double dg_dtheta_i_ = -std::cos(psi_i_) * std::sin(theta_i_) - std::cos(theta_i_) / d->therahold_slope;

//             //     // 填充对 psi_i 和 theta_i 的梯度
//             //     grad[(4 + i) * n + 0] += dg_dpsi_i_ * d->R[i](0, 0) + dg_dtheta_i_ * d->R[i](1, 0);
//             //     grad[(4 + i) * n + 1] += dg_dpsi_i_ * d->R[i](0, 1) + dg_dtheta_i_ * d->R[i](1, 1);
//             // }
//         } else {
//             result[4 + i] = 0.0;  // 对于无效约束，确保返回 0
//         }

//         std::cout << "Constraints: ";
//     for (unsigned i = 0; i < m; ++i) {
//         std::cout << result[i] << " ";
//     }
//     std::cout << std::endl;
//     }
// }



// // 约束函数
// void constraint_function2(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data) {
//     Data* d = static_cast<Data*>(data);
//     double psi_i = x[0];
//     double theta_i = x[1];

//     // 约束1: 检查 psi_i 和 theta_i 是否在 [psi_min, psi_max] 和 [theta_min, theta_max] 范围内
//     result[0] = psi_i - d->psi_min;  // psi_i >= psi_min
//     result[1] = d->psi_max - psi_i;  // psi_i <= psi_max
//     result[2] = theta_i - d->theta_min;  // theta_i >= theta_min
//     result[3] = d->theta_max - theta_i;  // theta_i <= theta_max

//     // 初始化梯度数组
//     if (grad) {
//         // 初始化梯度数组
//         std::fill(grad, grad + n * m, 0.0);  // 清空整个梯度数组

//         grad[0 * n + 0] = 1.0;  // 对应 result[0] 的 psi_i 偏导数
//         grad[1 * n + 0] = -1.0; // 对应 result[1] 的 psi_i 偏导数
//         grad[2 * n + 1] = 1.0;  // 对应 result[2] 的 theta_i 偏导数
//         grad[3 * n + 1] = -1.0; // 对应 result[3] 的 theta_i 偏导数
//     }

    
//          Eigen::Vector3f v = Angles2Velocity(psi_i, theta_i, d->V);

//         for (int i = 0; i < swarm_num_uav; i++) {
            
//                 result[4 + i] = 0;
//         }
// }

