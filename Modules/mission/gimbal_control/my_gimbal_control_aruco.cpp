/***************************************************************************************************************************
 *gimbal_control_landpad_track.cpp
 *
 * Author: k
 *
 * Update Time: 2024.6.27
 *
 * 说明: 吊仓追踪测试
 *      吊仓追踪逻辑：云台当前角度加上视角差的p控制
 * 
 * 追踪的时候当吊舱在移动时会有视觉估计误差？
***************************************************************************************************************************/
//ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include <deque>
#include <numeric>


#include "gimbal_control.h"
#include "mission_utils.h"
#include "message_utils.h"

// #include "gimbal.h"
#include "prometheus_gimbal_control/GimbalControl.h"
#include "prometheus_gimbal_control/GimbalState.h"
// #include "prometheus_msgs/ArucoInfo.h"
// #include "prometheus_gimbal_control/VisionDiff.h"

using namespace std;
using namespace Eigen;

#define NODE_NAME "my_gimbal_tracking_aruco"
#define PI 3.1415926

Eigen::Vector3d gimbal_att_sp;
Eigen::Vector3d gimbal_att;
Eigen::Vector3d gimbal_att_deg;
Eigen::Vector3d gimbal_att_rate;
Eigen::Vector3d gimbal_att_rate_deg;
Eigen::Vector3d gimbal_att_sp_deg;

Eigen::Matrix3f R_Body_to_ENU,R_camera_to_body;              // 无人机机体系至惯性系转换矩阵

nav_msgs::Odometry GroundTruth;             // 降落板真实位置（仿真中由Gazebo插件提供）
Eigen::Vector3d roi_point;
Eigen::Vector3d mav_pos_;
prometheus_msgs::ArucoInfo aruco_info;
Eigen::Vector3d aruco_pos_enu;
bool is_detected = false;
#define VISION_THRES_TRACKING 100

int num_regain = 0;
int num_lost = 0;

float sight_angle[2];
float gimbal_rate;
float k_gimbal_p, k_gimbal_i, k_gimbal_d, k_gimbal_p_vel;
double gimbal_window_size;
// 计算误差的变化率
float last_smoothed_pitch = 0.0;
float last_smoothed_yaw = 0.0;

float smoothed_pitch;
float smoothed_yaw;

double init_pitch;

Detection_result landpad_det;    
// prometheus_msgs::ArucoInfo aruco_info;
// bool use_pad_height;  // 是否使用降落板绝对高度
// float pad_height;
prometheus_msgs::DroneState _DroneState;    // 无人机状态
prometheus_gimbal_control::GimbalState gimbal_state_;
bool init_flag = false;

prometheus_gimbal_control::GimbalControl gimbal_msg_;


// #define WINDOW_SIZE 5 // 滑动窗口大小

std::deque<float> pitch_window;
std::deque<float> yaw_window;



float calculate_average(const std::deque<float>& window) {
    float sum = std::accumulate(window.begin(), window.end(), 0.0);
    return sum / window.size();
}

float calculate_sum(const std::deque<float>& window) {
    return std::accumulate(window.begin(), window.end(), 0.0);
}

void aruco_cb(const prometheus_msgs::ArucoInfo::ConstPtr &msg){
    aruco_info = *msg;
    
    

    if(aruco_info.detected)
    {
        num_regain++;
        num_lost = 0;
    }else
    {
        num_regain = 0;
        num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(num_lost > VISION_THRES)
    {
        aruco_info.detected = false;
        sight_angle[0] = 0.0;
        sight_angle[1] = 0.0;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(num_regain > VISION_THRES)
    {
        aruco_info.detected = true;
        sight_angle[0] = aruco_info.sight_angle[0];
        sight_angle[1] = aruco_info.sight_angle[1];

        
    }

    if(aruco_info.detected == true){
        Eigen::Vector3f pos_gimbal_frame;
        // 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
        // 目标坐标，机体向前x为正，向左y为正，向上z为正
        pos_gimbal_frame[0] =  aruco_info.position[1];
        pos_gimbal_frame[1] = - aruco_info.position[0];
        pos_gimbal_frame[2] = - aruco_info.position[2];
        // landpad_det.pos_body_frame =R_camera_to_body * pos_gimbal_frame;//从相机坐标系转换到机身坐标系

        // Eigen::Matrix3f R_camera_to_body_trans;
        // R_camera_to_body_trans << 1, 0, 0,
        //                     0, -1, 0,
        //                     0, 0, -1;
        
        //从机身看去，目标相对于机身前向为x正，左向为y正，上向为z正
        Eigen::Vector3f pos_body_frame = R_camera_to_body * pos_gimbal_frame;
        // std::cout<<"pos_gimbal_state[0] = "<< pos_gimbal_frame[0]<<" pos_gimbal_state[1] = "<<pos_gimbal_frame[1]<< " pos_gimbal_state[2] = "<< pos_gimbal_frame[2]<<std::endl;
        // std::cout<<"R_camera_to_body = "<< R_camera_to_body<< std::endl;

        // 目标相对与机体，前向为x正，右向为y正，下向为z正？？
        // landpad_det.pos_body_frame = R_camera_to_body_trans * landpad_det.pos_body_frame;

        // landpad_det.pos_body_enu_frame = landpad_det.pos_body_frame;
        // std::cout << "test02 R: " << R_camera_to_body << std::endl;

        Eigen::Vector3f pos_body_enu_frame = R_Body_to_ENU * landpad_det.pos_body_frame;
        
        // landpad_det.att_body_frame[0] = landpad_det.Detection_info.attitude[0];//yaw 顺时针为负
        // landpad_det.att_body_frame[1] = landpad_det.Detection_info.attitude[1];//pitch
        // landpad_det.att_body_frame[2] = landpad_det.Detection_info.attitude[2];//roll

    // std::cout<< "pos_body_frame[0] = " << pos_body_frame[0] << " pos_body_frame[1] = " << pos_body_frame[1] << " pos_body_frame[2] = " << pos_body_frame[2] << std::endl;
    // std::cout << "-------------------"  << std::endl;

    }else{
        // landpad_det.pos_body_enu_frame[0] = 0.0;
        // landpad_det.pos_body_enu_frame[1] = 0.0;
        // landpad_det.pos_body_enu_frame[2] = 0.0;
        // landpad_det.att_body_frame[0] = 0.0;//yaw 顺时针为负
        // landpad_det.att_body_frame[1] = 0.0;//pitch
        // landpad_det.att_body_frame[2] = 0.0;//roll
    }
    

    // std::cout<< "pos_body_enu_frame[0] = " << landpad_det.pos_body_enu_frame[0] << " pos_body_enu_frame[1] = " << landpad_det.pos_body_enu_frame[1] << " pos_body_enu_frame[2] = " << landpad_det.pos_body_enu_frame[2] << std::endl;

}


void gimbal_state_cb(const prometheus_gimbal_control::GimbalState::ConstPtr& msg){
    gimbal_state_ = *msg;//TODO
    //往右转yaw为正，往下转pitch为正，机体坐标系往右转roll为正，照这样计算向前x为正，向左y为正，向下z为正，显然是错的，因为这里yaw角是反的，应该是向上z为正.......
    // R_camera_to_body = my_get_rotation_matrix(gimbal_state_.rotorAngle[0], gimbal_state_.rotorAngle[1], - gimbal_state_.rotorAngle[2]);
    
    R_camera_to_body = get_rotation_matrix(gimbal_state_.rotorAngle[0] *(PI/180), gimbal_state_.rotorAngle[1] *(PI/180), - gimbal_state_.rotorAngle[2] *(PI/180));

    // cout << "test01 R: " << R_camera_to_body << std::endl;
    std::cout<<"gimbal_state_.rotorAngle[0] = " << gimbal_state_.rotorAngle[0]*(PI/180) << " gimbal_state_.rotorAngle[a] = " << gimbal_state_.rotorAngle[1]*(PI/180) << " gimbal_state_.rotorAngle[2] = " << gimbal_state_.rotorAngle[2]*(PI/180)<< std::endl;
    // Eigen::Vector3f target_angle;
    // target_angle[0] = gimbal_state_.rotorAngle[0];
    // target_angle[1] = gimbal_state_.rotorAngle[1] + sight_angle[0];
    // target_angle[2] = gimbal_state_.rotorAngle[2] + sight_angle[1];
    // R_camera_to_body = get_rotation_matrix(target_angle[0] *(PI/180), target_angle[1] *(PI/180), - target_angle[2] *(PI/180));

}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);

    mav_pos_ << _DroneState.position[0],_DroneState.position[1],_DroneState.position[2];

}

float constrain_angle(float angle) {
    // Reduce the angle to be within the range -180 to 180
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

// void gimbal_control_cb(const ros::TimerEvent& e) {
//     // 更新滑动窗口
//     if (pitch_window.size() >= gimbal_window_size) {
//         pitch_window.pop_front();
//     }
//     if (yaw_window.size() >= gimbal_window_size) {
//         yaw_window.pop_front();
//     }

//     pitch_window.push_back(sight_angle[1]);
//     yaw_window.push_back(sight_angle[0]);

//     // 计算平滑后的视场角误差P
//     smoothed_pitch = calculate_average(pitch_window);
//     smoothed_yaw = calculate_average(yaw_window);

//     // 计算平滑后的视场角误差I
//     float pitch_integral = calculate_sum(pitch_window);
//     float yaw_integral = calculate_sum(yaw_window);
//     float max_i = 0.1;
//     pitch_integral = min(pitch_integral, max_i);
//     yaw_integral = min(yaw_integral, max_i);

//     // 计算平滑后的视场角误差D
//     float pitch_error_rate = 0.0;
//     float yaw_error_rate = 0.0;

//     pitch_error_rate = smoothed_pitch - last_smoothed_pitch;
//     yaw_error_rate = smoothed_yaw - last_smoothed_yaw;

//     last_smoothed_pitch = smoothed_pitch;
//     last_smoothed_yaw = smoothed_yaw;


    
//     // gimbal_msg_.rpyMode = prometheus_gimbal_control::GimbalControl::manual;
//     // gimbal_msg_.roll = prometheus_gimbal_control::GimbalControl::angleCtl;
//     // gimbal_msg_.pitch = prometheus_gimbal_control::GimbalControl::angleCtl;
//     // gimbal_msg_.yaw = prometheus_gimbal_control::GimbalControl::angleCtl;
//     // gimbal_msg_.rValue = 0.0;
//     // // gimbal_msg_.pValue = gimbal_state_.rotorAngle[1] + (k_gimbal_p * smoothed_pitch + k_gimbal_d * pitch_error_rate) / PI * 180;
//     // // gimbal_msg_.yValue = gimbal_state_.rotorAngle[2] + (k_gimbal_p * smoothed_yaw + k_gimbal_d * yaw_error_rate) / PI * 180;
//     // gimbal_msg_.pValue = 0.0;
//     // gimbal_msg_.yValue = 0.0;

//     // std::cout << "smoothed_pitch = " << smoothed_pitch << std::endl;
//     gimbal_msg_.rpyMode = prometheus_gimbal_control::GimbalControl::manual;
//     gimbal_msg_.roll = prometheus_gimbal_control::GimbalControl::velocityCtl;
//     gimbal_msg_.pitch = prometheus_gimbal_control::GimbalControl::velocityCtl;
//     gimbal_msg_.yaw = prometheus_gimbal_control::GimbalControl::velocityCtl;
//     gimbal_msg_.rValue = 0.0;
//     gimbal_msg_.pValue = k_gimbal_p_vel * smoothed_pitch;
//     gimbal_msg_.yValue = k_gimbal_p_vel * smoothed_yaw;



// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_control_landpad_track");
    ros::NodeHandle nh("~");

    // 节点运行频率： 10hz 
    ros::Rate rate(30.0);


    nh.param<float>("gimbal_rate", gimbal_rate, 0.1);
    nh.param<float>("k_gimbal_p", k_gimbal_p, 0.1 );
    nh.param<float>("k_gimbal_i", k_gimbal_i, 0.1 );
    nh.param<float>("k_gimbal_d", k_gimbal_d, 0.5 );

    nh.param<float>("k_gimbal_p_vel", k_gimbal_p_vel, 10.0 );

    nh.param<double>("init_pitch", init_pitch, 30.0);
    nh.param<double>("gimbal_window_size", gimbal_window_size, 5);

    // gimbal_control gimbal_control_;

    // ros::Subscriber landpad_det_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/landpad_det", 10, landpad_det_cb);
    ros::Subscriber aruco_sub = nh.subscribe<prometheus_msgs::ArucoInfo>("/prometheus/object_detection/aruco_det", 10, aruco_cb);
    
    //【订阅】无人机状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    ros::Subscriber gimbal_state_sub = nh.subscribe<prometheus_gimbal_control::GimbalState>("/gimbal/state", 10, gimbal_state_cb);

    //吊舱控制timer
    // ros::Timer timer = nh.createTimer(ros::Duration(gimbal_rate), gimbal_control_cb);

    
    // 创建一个发布者，用于发布吊舱控制命令到 /gimbal/control 话题
    ros::Publisher gimbal_control_pub = nh.advertise<prometheus_gimbal_control::GimbalControl>("/gimbal/control", 10);

    while(ros::ok() )
    {
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

        
        if(aruco_info.detected == true){


            // 更新滑动窗口
            if (pitch_window.size() >= gimbal_window_size) {
                pitch_window.pop_front();
            }
            if (yaw_window.size() >= gimbal_window_size) {
                yaw_window.pop_front();
            }

            pitch_window.push_back(sight_angle[1]);
            yaw_window.push_back(sight_angle[0]);

            // 计算平滑后的视场角误差P
            smoothed_pitch = calculate_average(pitch_window);
            smoothed_yaw = calculate_average(yaw_window);

            // 计算平滑后的视场角误差I
            float pitch_integral = calculate_sum(pitch_window);
            float yaw_integral = calculate_sum(yaw_window);
            float max_i = 0.1;
            pitch_integral = min(pitch_integral, max_i);
            yaw_integral = min(yaw_integral, max_i);

            // 计算平滑后的视场角误差D
            float pitch_error_rate = 0.0;
            float yaw_error_rate = 0.0;

            pitch_error_rate = smoothed_pitch - last_smoothed_pitch;
            yaw_error_rate = smoothed_yaw - last_smoothed_yaw;

            last_smoothed_pitch = smoothed_pitch;
            last_smoothed_yaw = smoothed_yaw;


            
            // gimbal_msg_.rpyMode = prometheus_gimbal_control::GimbalControl::manual;
            // gimbal_msg_.roll = prometheus_gimbal_control::GimbalControl::angleCtl;
            // gimbal_msg_.pitch = prometheus_gimbal_control::GimbalControl::angleCtl;
            // gimbal_msg_.yaw = prometheus_gimbal_control::GimbalControl::angleCtl;
            // gimbal_msg_.rValue = 0.0;
            // // gimbal_msg_.pValue = gimbal_state_.rotorAngle[1] + (k_gimbal_p * smoothed_pitch + k_gimbal_d * pitch_error_rate) / PI * 180;
            // // gimbal_msg_.yValue = gimbal_state_.rotorAngle[2] + (k_gimbal_p * smoothed_yaw + k_gimbal_d * yaw_error_rate) / PI * 180;
            // gimbal_msg_.pValue = 0.0;
            // gimbal_msg_.yValue = 0.0;

            // std::cout << "smoothed_pitch = " << smoothed_pitch << std::endl;
            gimbal_msg_.rpyMode = prometheus_gimbal_control::GimbalControl::manual;
            gimbal_msg_.roll = prometheus_gimbal_control::GimbalControl::velocityCtl;
            gimbal_msg_.pitch = prometheus_gimbal_control::GimbalControl::velocityCtl;
            gimbal_msg_.yaw = prometheus_gimbal_control::GimbalControl::velocityCtl;
            gimbal_msg_.rValue = 0.0;
            gimbal_msg_.pValue = k_gimbal_p_vel * smoothed_pitch;
            gimbal_msg_.yValue = k_gimbal_p_vel * smoothed_yaw;


            gimbal_control_pub.publish(gimbal_msg_);

        }else{

            gimbal_msg_.rpyMode = prometheus_gimbal_control::GimbalControl::manual;
            gimbal_msg_.roll = prometheus_gimbal_control::GimbalControl::angleCtl;
            gimbal_msg_.pitch = prometheus_gimbal_control::GimbalControl::angleCtl;
            gimbal_msg_.yaw = prometheus_gimbal_control::GimbalControl::angleCtl;
            gimbal_msg_.rValue = 0.0;
            gimbal_msg_.pValue = 0.0;
            gimbal_msg_.yValue = 0.0;
            gimbal_control_pub.publish(gimbal_msg_);
        }

        ros::spinOnce();
        rate.sleep();
    }
        
}