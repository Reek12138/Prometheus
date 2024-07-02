/***************************************************************************************************************************
 *gimbal_control_landpad_track.cpp
 *
 * Author: k
 *
 * Update Time: 2024.6.27
 *
 * 说明: 吊仓追踪测试
 *      吊仓追踪逻辑：云台当前角度加上视角差的p控制
***************************************************************************************************************************/
//ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include <deque>
#include <numeric>


#include "gimbal_control.h"
#include "mission_utils.h"
#include "message_utils.h"

using namespace std;
using namespace Eigen;

#define NODE_NAME "gimbal_control_landpad_track"
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
float k_gimbal_p, k_gimbal_i, k_gimbal_d;
double gimbal_window_size;
// 计算误差的变化率
float last_smoothed_pitch = 0.0;
float last_smoothed_yaw = 0.0;

Detection_result landpad_det;    
// bool use_pad_height;  // 是否使用降落板绝对高度
// float pad_height;
prometheus_msgs::DroneState _DroneState;    // 无人机状态
bool init_flag = false;



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

void groundtruth_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    GroundTruth = *msg;

    roi_point[0] = GroundTruth.pose.pose.position.x;
    roi_point[1] = GroundTruth.pose.pose.position.y;
    roi_point[2] = GroundTruth.pose.pose.position.z;
}
void landpad_det_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg){
    landpad_det.object_name = "landpad";
    landpad_det.Detection_info = *msg;
    // 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    // 相机安装误差 在mission_utils.h中设置
    // landpad_det.pos_body_frame[0] = - landpad_det.Detection_info.position[1] + camera_offset[0];
    // landpad_det.pos_body_frame[1] = - landpad_det.Detection_info.position[0] + camera_offset[1];
    // landpad_det.pos_body_frame[2] = - landpad_det.Detection_info.position[2] + camera_offset[2];
    Eigen::Vector3f pos_gimbal_frame;
    pos_gimbal_frame[0] = - landpad_det.Detection_info.position[1];
    pos_gimbal_frame[1] = - landpad_det.Detection_info.position[0];
    pos_gimbal_frame[2] = - landpad_det.Detection_info.position[2];

    landpad_det.pos_body_frame =R_camera_to_body * pos_gimbal_frame;//从相机坐标系转换到机身坐标系

    // 机体系 -> 机体惯性系 (原点在机体的惯性系) (对无人机姿态进行解耦)
    // landpad_det.pos_body_enu_frame = R_Body_to_ENU * landpad_det.pos_body_frame;
    landpad_det.pos_body_enu_frame = landpad_det.pos_body_frame;
    std::cout<< "pos_body_enu_frame[0] = " << landpad_det.pos_body_enu_frame[0] << " pos_body_enu_frame[1] = " << landpad_det.pos_body_enu_frame[1] << " pos_body_enu_frame[2] = " << landpad_det.pos_body_enu_frame[2] << std::endl;
    Eigen::Vector3d error_vec;
    error_vec = roi_point - mav_pos_;
    std::cout<< "true_error[0] = " << error_vec[0] << "  true_error[1] = "<< error_vec[1] << " true_error[2] = " << error_vec[2] << std::endl ;
    
    landpad_det.att_body_frame[0] = landpad_det.Detection_info.attitude[0];//yaw 顺时针为负
    landpad_det.att_body_frame[1] = landpad_det.Detection_info.attitude[1];//pitch
    landpad_det.att_body_frame[2] = landpad_det.Detection_info.attitude[2];//roll 

    
    // 机体系 -> 机体惯性系 (原点在机体的惯性系) (对无人机姿态进行解耦)
    // landpad_det.pos_body_enu_frame = R_Body_to_ENU * landpad_det.pos_body_frame;

    // 机体惯性系 -> 惯性系
    // landpad_det.pos_enu_frame[0] = _DroneState.position[0] + landpad_det.pos_body_enu_frame[0];
    // landpad_det.pos_enu_frame[1] = _DroneState.position[1] + landpad_det.pos_body_enu_frame[1];
    // landpad_det.pos_enu_frame[2] = _DroneState.position[2] + landpad_det.pos_body_enu_frame[2];
    // 此降落方案不考虑偏航角 （高级版可提供）
    // landpad_det.att_enu_frame[2] = 0.0;

    if(landpad_det.Detection_info.detected)
    {
        landpad_det.num_regain++;
        landpad_det.num_lost = 0;
    }else
    {
        landpad_det.num_regain = 0;
        landpad_det.num_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(landpad_det.num_lost > VISION_THRES)
    {
        landpad_det.is_detected = false;
        sight_angle[0] = 0.0;
        sight_angle[1] = 0.0;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(landpad_det.num_regain > VISION_THRES)
    {
        landpad_det.is_detected = true;
        sight_angle[0] = landpad_det.Detection_info.sight_angle[0];
        sight_angle[1] = landpad_det.Detection_info.sight_angle[1];
    }
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

void gimbal_control_cb(const ros::TimerEvent& e) {
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
    float smoothed_pitch = calculate_average(pitch_window);
    float smoothed_yaw = calculate_average(yaw_window);

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


    // if (pitch_window.size() > 1) {
    //     pitch_error_rate = (pitch_window.back() - pitch_window.front()) / (pitch_window.size() * gimbal_rate);
    // }
    // if (yaw_window.size() > 1) {
    //     yaw_error_rate = (yaw_window.back() - yaw_window.front()) / (yaw_window.size() * gimbal_rate);
    // }


    // 使用平滑后的视场角误差进行控制
    // gimbal_att_sp[0] = 0.0;
    // gimbal_att_sp[1] = gimbal_att_deg[1] - k_gimbal_p * smoothed_pitch / PI * 180; // pitch
    // gimbal_att_sp[1] = max(gimbal_att_sp[1], -60.0);
    // // gimbal_att_sp[2] = gimbal_att_deg[2] + k_gimbal_p * smoothed_yaw / PI * 180; // yaw
    // gimbal_att_sp[2] = 0.0; // yaw

    gimbal_att_sp[0] = 0.0;
    // gimbal_att_sp[1] = gimbal_att_deg[1] - (k_gimbal_p * smoothed_pitch + k_gimbal_d * pitch_error_rate) / PI * 180; // pitch
    // gimbal_att_sp[1] = max(gimbal_att_sp[1], -60.0);
    gimbal_att_sp[1] = -60.0;
    gimbal_att_sp[2] = 0.0; // yaw
    // gimbal_att_sp[2] = gimbal_att_deg[2] + (k_gimbal_p * smoothed_yaw + k_gimbal_d * yaw_error_rate) / PI * 180; // yaw

    // gimbal_att_sp[0] = 0.0;
    // gimbal_att_sp[1] = gimbal_att_deg[1] - (k_gimbal_p * smoothed_pitch + k_gimbal_i * pitch_integral + k_gimbal_d * pitch_error_rate) / PI * 180; // pitch
    // gimbal_att_sp[2] = gimbal_att_deg[2] + (k_gimbal_p * smoothed_yaw + k_gimbal_i * yaw_integral + k_gimbal_d * yaw_error_rate) / PI * 180; // yaw


//     gimbal_att_sp[0] = 0;
    
//     float pitch_angle = gimbal_att_deg[1] - smoothed_pitch / PI * 180;
//     if (pitch_angle >= -30.0 && pitch_angle < 0.0) {
//         gimbal_att_sp[1] = -15.0;
//     } else if (pitch_angle >= -60.0 && pitch_angle < -30.0) {
//         gimbal_att_sp[1] = -45.0;
//     }else {
//         gimbal_att_sp[1] = -60.0;
//     }
    
//     float yaw_angle = constrain_angle(gimbal_att_deg[2] + smoothed_yaw / PI * 180);

//     if (yaw_angle >= -30.0 && yaw_angle < 30.0) {
//     gimbal_att_sp[2] = 0.0;
// } else if (yaw_angle >= -90.0 && yaw_angle < -30.0) {
//     gimbal_att_sp[2] = -60.0;
// } else if (yaw_angle >= -150.0 && yaw_angle < -90.0) {
//     gimbal_att_sp[2] = -120.0;
// } else if (yaw_angle >= -180.0 && yaw_angle < -150.0) {
//     gimbal_att_sp[2] = -180.0;
// } else if (yaw_angle >= 30.0 && yaw_angle < 90.0) {
//     gimbal_att_sp[2] = 60.0;
// } else if (yaw_angle >= 90.0 && yaw_angle < 150.0) {
//     gimbal_att_sp[2] = 120.0;
// } else if (yaw_angle >= 150.0 && yaw_angle <= 180.0) {
//     gimbal_att_sp[2] = 180.0;
// }
    
    

    // gimbal_control_.send_mount_control_command(gimbal_att_sp);
}

void aruco_cb(const prometheus_msgs::ArucoInfo::ConstPtr& msg)
{
    aruco_info = *msg;

    // 暂不考虑无人机姿态的影响
    aruco_pos_enu[0] = mav_pos_[0] - aruco_info.position[1];
    aruco_pos_enu[1] = mav_pos_[1] - aruco_info.position[0];
    // 相机安装在无人机下方10cm处，需减去该偏差
    aruco_pos_enu[2] = mav_pos_[2] - aruco_info.position[2];

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
    if(num_lost > VISION_THRES_TRACKING)
    {
        is_detected = false;

        //　丢失后　对sight_angle清零，否则云台会移动
        sight_angle[0] = 0.0;
        sight_angle[1] = 0.0;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(num_regain > 2)
    {
        is_detected = true;
    }

    if(aruco_info.detected)
    {
        cout << "Aruco_ID: [" << aruco_info.aruco_num << "]  detected: [yes] " << endl;
        cout << "Pos [camera]: "<< aruco_info.position[0] << " [m] "<< aruco_info.position[1] << " [m] "<< aruco_info.position[2] << " [m] "<<endl;
        cout << "Pos [enu]   : "<< aruco_pos_enu[0]       << " [m] "<< aruco_pos_enu[1]       << " [m] "<< aruco_pos_enu[2]       << " [m] "<<endl;
        // cout << "Att [camera]: "<< aruco_info.position[0] << " [m] "<< aruco_info.position[1] << " [m] "<< aruco_info.position[2] << " [m] "<<endl;
        cout << "Sight Angle : "<< aruco_info.sight_angle[0]/3.14*180 << " [deg] "<< aruco_info.sight_angle[1]/3.14*180 << " [deg] " <<endl;
    }else
    {
        cout << "Aruco_ID: [" << aruco_info.aruco_num << "]  detected: [no] " << endl;
    }
    

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_control_landpad_track");
    ros::NodeHandle nh("~");

    // 节点运行频率： 10hz 
    ros::Rate rate(10.0);


    nh.param<float>("gimbal_rate", gimbal_rate, 0.1);
    nh.param<float>("k_gimbal_p", k_gimbal_p, 0.5 );
    nh.param<float>("k_gimbal_i", k_gimbal_i, 0.1 );
    nh.param<float>("k_gimbal_d", k_gimbal_d, 0.5 );
    nh.param<double>("gimbal_window_size", gimbal_window_size, 5);

    gimbal_control gimbal_control_;

    ros::Subscriber landpad_det_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/landpad_det", 10, landpad_det_cb);
    // ros::Subscriber aruco_sub = nh.subscribe<prometheus_msgs::ArucoInfo>("/prometheus/object_detection/aruco_det", 10, aruco_cb);
    
    
    //【订阅】无人机状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //吊舱控制timer
    ros::Timer timer = nh.createTimer(ros::Duration(gimbal_rate), gimbal_control_cb);

    //  //【订阅】地面真值，此信息仅做比较使用 不强制要求提供
    ros::Subscriber groundtruth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/landing_pad", 10, groundtruth_cb);
    //【订阅】地面真值，此信息仅做比较使用 不强制要求提供
    // ros::Subscriber groundtruth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/marker", 10, groundtruth_cb);

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

        gimbal_att = gimbal_control_.get_gimbal_att();
        gimbal_att_deg = gimbal_att/PI*180;
        cout << "gimbal_att    : " << gimbal_att_deg[0] << " [deg] "<< gimbal_att_deg[1] << " [deg] "<< gimbal_att_deg[2] << " [deg] "<<endl;
        gimbal_att_rate = gimbal_control_.get_gimbal_att_rate();
        gimbal_att_rate_deg = gimbal_att_rate/PI*180;
        // cout << "gimbal_att_rate    : " << gimbal_att_rate_deg[0] << " [deg/s] "<< gimbal_att_rate_deg[1] << " [deg/s] "<< gimbal_att_rate_deg[2] << " [deg/s] "<<endl;
        R_camera_to_body = get_rotation_matrix(gimbal_att[0], gimbal_att[1], gimbal_att[2]);

        if(landpad_det.is_detected == true){
        // if(is_detected == true){
            // double distance_2d = std::sqrt(landpad_det.pos_body_enu_frame[0] * landpad_det.pos_body_enu_frame[0] +
            //                                                                 landpad_det.pos_body_enu_frame[1] * landpad_det.pos_body_enu_frame[1]);
            // gimbal_att_sp_deg[0] = 0.0;
            // // gimbal_att_sp_deg[1] = -60.0;
            // // gimbal_att_sp_deg[0] = 0.0;
            // gimbal_att_sp_deg[1] = std::atan2(landpad_det.pos_body_enu_frame[2], distance_2d)/PI*180;
            // gimbal_att_sp_deg[2] = 180 - std::atan2(landpad_det.pos_body_enu_frame[1], landpad_det.pos_body_enu_frame[0])/PI*180;
            gimbal_control_.send_mount_control_command(gimbal_att_sp);
        }

        ros::spinOnce();
        rate.sleep();
    }
        
}