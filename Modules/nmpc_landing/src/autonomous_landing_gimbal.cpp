/***************************************************************************************************************************
 * autonomous_landing.cpp
 *
 * Author: k
 *
 * Update Time: 2024.4.17
 *
 * 说明: 自主降落程序（支持Gazebo仿真 + 静态/慢速靶标真实实验, 直线测试)
 *      1. 订阅目标位置(来自视觉的ros节点)
 *      2. 追踪算法及追踪策略
 *      3. 发布上层控制指令 (prometheus_msgs::ControlCommand)
***************************************************************************************************************************/
//ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>

#include "mission_utils.h"
#include "message_utils.h"
#include "nmpc_ctr.h"
#include "gimbal_control.h"


using namespace std;
using namespace Eigen;

#define NODE_NAME "autonomous_landing_gimbal"
#define PI 3.1415926

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool hold_mode; // 悬停模式，用于测试检测精度
bool sim_mode;  // 选择Gazebo仿真模式 或 真实实验模式
bool use_pad_height;  // 是否使用降落板绝对高度
float pad_height;
string message;
std_msgs::Bool vision_switch;
geometry_msgs::PoseStamped mission_cmd;
float start_point[3];    // 起始降落位置
float camera_offset[3];
bool moving_target;
float target_vel_xy[2];         // 目标移动速度 enu坐标系 单位：m/s
std_msgs::Bool flag_start;
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::DroneState _DroneState;    // 无人机状态
Eigen::Matrix3f R_Body_to_ENU;              // 无人机机体系至惯性系转换矩阵
Eigen::Matrix3f R_camera_to_body;        //吊仓到机体的转换矩阵
//---------------------------------------Vision---------------------------------------------
nav_msgs::Odometry GroundTruth;             // 降落板真实位置（仿真中由Gazebo插件提供）
Detection_result landpad_det;               // 检测结果
//---------------------------------------Track---------------------------------------------
float kp_land[3];         //控制参数 - 比例参数
float kd_land[3];           //控制参数 - 微分参数

float dynamic_kp_land[3];         //控制参数 - 比例参数
float dynamic_kd_land[3];           //控制参数 - 微分参数

// -------------------------------gimbal_tracking-----------------------------------
float sight_angle[2];
float k_gimbal_p, k_gimbal_i, k_gimbal_d;
double gimbal_window_size;

std::deque<float> pitch_window;
std::deque<float> yaw_window;

Eigen::Vector3d gimbal_att_sp;
Eigen::Vector3d gimbal_att_sp_init(0.0,-70.0,0.0);
Eigen::Vector3d gimbal_att_sp_lock(0.0,-90.0,0.0);
Eigen::Vector3d gimbal_att;
Eigen::Vector3d gimbal_att_deg;
Eigen::Vector3d gimbal_att_rate;
Eigen::Vector3d gimbal_att_rate_deg;
Eigen::Vector3d gimbal_att_sp_deg;

// 计算误差的变化率
float last_smoothed_pitch = 0.0;
float last_smoothed_yaw = 0.0;

float gimbal_rate;


Eigen::Vector3f last_tracking_position = Eigen::Vector3f::Zero();
ros::Time last_tracking_time;
Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

Eigen::Vector3f last_tracking_position2 = Eigen::Vector3f::Zero();
ros::Time last_tracking_time2;
Eigen::Vector3f velocity2 = Eigen::Vector3f::Zero();

bool is_tracking_initialized = false;
bool dynamic_tracking_initialized = false;

double average_velocity_time;
double D_time;

float k;//速度补偿参数
float k2;//距离补偿参数TODO:这里视觉测距和真实测距有成倍的误差

float dynamic_distance ;//切换动态降落的距离
float dynamic_height ;//切换动态降落的高度

Eigen::Vector2f current_states;//无人机当前xy
Eigen::Vector2f landing_pad_states;//landing_pad当前xy

float true_target_vel[2];//测试用,目标速度真值

// 视觉检测目标速度计算变量
Eigen::Vector3f vel_sum(0.0, 0.0, 0.0);
int num = 0;

// 滑动窗口
std::deque<Eigen::Vector3f> buffer;
int window_size;

float delta;

Eigen::Vector3d roi_point;
Eigen::Vector3d mav_pos_;
bool ignore_version = true;




// 五种状态机
enum EXEC_STATE
{
    WAITING_RESULT,
    TRACKING,
    DYNAMIC_LANDING,
    LOST,
    LANDING,
};
EXEC_STATE exec_state;

float distance_to_pad;
float arm_height_to_ground;
float arm_distance_to_pad;
//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();                                                                 //打印各项参数以供检查
void printf_result();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void landpad_det_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
// """
// 目标的三维位置 (pose_now.position): [x, y, z]
// 目标的姿态（欧拉角）(pose_now.attitude): [roll, pitch, yaw]
// 目标的姿态（四元数）(pose_now.attitude_q): [qx, qy, qz, qw]
// 视角角度 (pose_now.sight_angle): [angle_x, angle_y]
// 偏航误差 (pose_now.yaw_error)
// """
{
    landpad_det.object_name = "landpad";
    landpad_det.Detection_info = *msg;
    // 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    // 相机安装误差 在mission_utils.h中设置
    Eigen::Vector3f pos_gimbal_frame;
    pos_gimbal_frame[0] = - landpad_det.Detection_info.position[1];
    pos_gimbal_frame[1] = - landpad_det.Detection_info.position[0];
    pos_gimbal_frame[2] = - landpad_det.Detection_info.position[2];

    Eigen::Vector3f pos_body_frame_no_offset = R_camera_to_body * pos_gimbal_frame;
    landpad_det.pos_body_frame[0] = pos_body_frame_no_offset[0] + camera_offset[0];
    landpad_det.pos_body_frame[1] = pos_body_frame_no_offset[1] + camera_offset[1];
    landpad_det.pos_body_frame[2] = pos_body_frame_no_offset[2] + camera_offset[2];

    // landpad_det.pos_body_enu_frame = R_Body_to_ENU * landpad_det.pos_body_frame;
    landpad_det.pos_body_enu_frame = landpad_det.pos_body_frame;

    // landpad_det.pos_body_frame[0] = - landpad_det.Detection_info.position[1] + camera_offset[0];
    // landpad_det.pos_body_frame[1] = - landpad_det.Detection_info.position[0] + camera_offset[1];
    // landpad_det.pos_body_frame[2] = - landpad_det.Detection_info.position[2] + camera_offset[2];
    landpad_det.att_body_frame[0] = landpad_det.Detection_info.attitude[0];//yaw 顺时针为负
    landpad_det.att_body_frame[1] = landpad_det.Detection_info.attitude[1];//pitch
    landpad_det.att_body_frame[2] = landpad_det.Detection_info.attitude[2];//roll 

    
    // 机体系 -> 机体惯性系 (原点在机体的惯性系) (对无人机姿态进行解耦)
    // landpad_det.pos_body_enu_frame = R_Body_to_ENU * landpad_det.pos_body_frame;

    if(use_pad_height)
    {
        //若已知降落板高度，则无需使用深度信息。
        landpad_det.pos_body_enu_frame[2] = pad_height - _DroneState.position[2];
    }

    // 机体惯性系 -> 惯性系
    landpad_det.pos_enu_frame[0] = _DroneState.position[0] + landpad_det.pos_body_enu_frame[0];
    landpad_det.pos_enu_frame[1] = _DroneState.position[1] + landpad_det.pos_body_enu_frame[1];
    landpad_det.pos_enu_frame[2] = _DroneState.position[2] + landpad_det.pos_body_enu_frame[2];
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
    // gimbal_control gimbal_control_;

    _DroneState = *msg;

    R_Body_to_ENU = get_rotation_matrix(_DroneState.attitude[0], _DroneState.attitude[1], _DroneState.attitude[2]);

    mav_pos_ << _DroneState.position[0],_DroneState.position[1],_DroneState.position[2];


    // gimbal_att = gimbal_control_.get_gimbal_att();
    // R_camera_to_body = get_rotation_matrix(gimbal_att[0], gimbal_att[1], gimbal_att[2]);

}

void groundtruth_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    GroundTruth = *msg;
    roi_point[0] = GroundTruth.pose.pose.position.x;
    roi_point[1] = GroundTruth.pose.pose.position.y;
    roi_point[2] = GroundTruth.pose.pose.position.z;

    // landing_pad_states << roi_point[0], roi_point[1];
}

void switch_cb(const std_msgs::Bool::ConstPtr& msg)
{
    flag_start = *msg;
}

void mission_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    mission_cmd = *msg;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>my function>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// 计算一秒内目标的平均相对速度
void timerCallback(const ros::TimerEvent&)
{
    if (is_tracking_initialized) {
        ros::Time current_time = ros::Time::now();
        float dt = (current_time - last_tracking_time).toSec();
        ROS_INFO_STREAM("current_time:"<<current_time);
        // ROS_INFO_STREAM("last_time:"<<last_tracking_time);
        if (dt > 0) {
            // Eigen::Vector3f current_position = landpad_det.pos_body_enu_frame;

            Eigen::Vector3f current_position ;
            current_position << roi_point[0] - _DroneState.position[0],
                                                    roi_point[1] - _DroneState.position[1],
                                                    roi_point[2] - _DroneState.position[2];

            velocity = (current_position - last_tracking_position) / dt;
            // vel_sum += velocity;
            // num +=1;
            // Eigen::Vector3f ave_velocity = vel_sum/num;
            ROS_INFO_STREAM("Current dist Position: " << current_position.transpose());
            ROS_INFO_STREAM("Current drone Position(true value): " << current_states.transpose());
            ROS_INFO_STREAM("Current landing_pad Position(true value): " << landing_pad_states.transpose());
            ROS_INFO_STREAM("Last Tracking Position: " << last_tracking_position.transpose());
            ROS_INFO_STREAM("Delta Time (dt): " << dt);
            ROS_INFO_STREAM("Velocity: " << velocity.transpose());
            // ROS_INFO_STREAM("Ave_Velocity: " << ave_velocity.transpose());
            last_tracking_position = current_position;
            last_tracking_time = current_time;
        }
    }
}

void D_cb(const ros::TimerEvent&){
    if (is_tracking_initialized) {
        ros::Time current_time2 = ros::Time::now();
        float dt2 = (current_time2 - last_tracking_time2).toSec();
        if (dt2 > 0) {
            Eigen::Vector3f current_position2 = landpad_det.pos_body_enu_frame;
            velocity2 = (current_position2 - last_tracking_position2) / dt2;
            // ROS_INFO_STREAM("Current Position: " << current_position.transpose());
            // ROS_INFO_STREAM("Last Tracking Position: " << last_tracking_position.transpose());
            // ROS_INFO_STREAM("Delta Time (dt): " << dt);
            // ROS_INFO_STREAM("Velocity: " << velocity.transpose());
            last_tracking_position2 = current_position2;
            last_tracking_time2 = current_time2;
        }
    }
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    double x,y,z;
    double roll,pitch,yaw;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;
    //四元数转欧拉角
    tf::Quaternion quat;                                     //定义一个四元数
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat); //取出方向存储于四元数
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    current_states << x,y;
}

void landing_pad_cb(const nav_msgs::Odometry::ConstPtr& msg){
    double x,y,z;
    double roll,pitch,yaw;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;
    //四元数转欧拉角
    tf::Quaternion quat;                                     //定义一个四元数
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat); //取出方向存储于四元数
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    landing_pad_states << x,y;
}

float calculate_average(const std::deque<float>& window) {
    float sum = std::accumulate(window.begin(), window.end(), 0.0);
    return sum / window.size();
}
void gimbal_control_cb(const ros::TimerEvent& e) {
    if(ignore_version){
        Eigen::Vector3d error_vec;
        double distance_2d;
        error_vec = roi_point - mav_pos_;
        distance_2d = std::sqrt(error_vec(0) * error_vec(0) + error_vec(1) * error_vec(1));
        // 理想的吊舱控制情况
        gimbal_att_sp[0] = 0.0;
        gimbal_att_sp[1] = std::atan2(error_vec(2), distance_2d)/PI*180; //pitch
        //desired_yaw = -std::atan2(error_vec(1), error_vec(0))/PI*180;//yaw
        gimbal_att_sp[2] =  -std::atan2(error_vec(1), error_vec(0))/PI*180;
    }else{
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

            // 计算平滑后的视场角误差D
            float pitch_error_rate = 0.0;
            float yaw_error_rate = 0.0;

            pitch_error_rate = smoothed_pitch - last_smoothed_pitch;
            yaw_error_rate = smoothed_yaw - last_smoothed_yaw;

            last_smoothed_pitch = smoothed_pitch;
            last_smoothed_yaw = smoothed_yaw;

            // 使用平滑后的视场角误差进行控制
            gimbal_att_sp[0] = 0.0;
            gimbal_att_sp[1] = gimbal_att_deg[1] - k_gimbal_p * smoothed_pitch / PI * 180; // pitch
            // gimbal_att_sp[2] = gimbal_att_deg[2] + k_gimbal_p * smoothed_yaw / PI * 180; // yaw
            gimbal_att_sp[2] = 0.0; // yaw
    }
    
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonomous_landing_gimbal");
    ros::NodeHandle nh("~");

    // 节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(20.0);

    //【订阅】降落板与无人机的相对位置及相对偏航角  单位：米   单位：弧度
    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    //  标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    ros::Subscriber landpad_det_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/landpad_det", 10, landpad_det_cb);

    //【订阅】无人机状态
    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //【订阅】地面真值，此信息仅做比较使用 不强制要求提供
    ros::Subscriber groundtruth_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/landing_pad", 10, groundtruth_cb);

    //【订阅】用于中断任务，直接降落
    ros::Subscriber mission_sub = nh.subscribe<geometry_msgs::PoseStamped>("/prometheus/mission/cmd", 10, mission_cb);

    //【订阅】降落程序开关，默认情况下不启用，用于多任务情况
    ros::Subscriber switch_sub = nh.subscribe<std_msgs::Bool>("/prometheus/switch/landing", 10, switch_cb);

    // 【发布】 视觉模块开关量
    ros::Publisher vision_switch_pub = nh.advertise<std_msgs::Bool>("/prometheus/switch/ellipse_det", 10);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);

    // 『定时器』 用于计算一段时间内目标的相对速度
    ros::Timer T = nh.createTimer(ros::Duration(average_velocity_time), timerCallback);
    // ros::Timer T = nh.createTimer(ros::Duration(1.0), timerCallback);

    // 『定时器』用于计算d参数
    ros::Timer T2 = nh.createTimer(ros::Duration(D_time),D_cb);

    // 【订阅】无人机的odom
    ros::Subscriber odom_listener = nh.subscribe<nav_msgs::Odometry>("/prometheus/drone_odom",10,odom_cb);

    //【订阅】地面真值，此信息仅做比较使用 不强制要求提供
    ros::Subscriber groundtruth_sub2 = nh.subscribe<nav_msgs::Odometry>("/ground_truth/landing_pad", 10, landing_pad_cb);

    //【控制器】NMPC控制器
    int predict_step = 50;
    float sample_time = 0.1;
    NMPC nmpc_ctr(predict_step, sample_time);

    //吊舱控制timer
    ros::Timer timer = nh.createTimer(ros::Duration(gimbal_rate), gimbal_control_cb);
    gimbal_control gimbal_control_;
    gimbal_att = gimbal_control_.get_gimbal_att();
    gimbal_att_deg = gimbal_att/PI*180;
    R_camera_to_body = get_rotation_matrix(gimbal_att[0], gimbal_att[1], gimbal_att[2]);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    //强制上锁高度
    nh.param<float>("arm_height_to_ground", arm_height_to_ground, 0.4);
    //强制上锁距离
    nh.param<float>("arm_distance_to_pad", arm_distance_to_pad, 0.3);
    // 悬停模式 - 仅用于观察检测结果
    nh.param<bool>("hold_mode", hold_mode, false);
    // 仿真模式 - 区别在于是否自动切换offboard模式
    nh.param<bool>("sim_mode", sim_mode, true);
    // 是否使用降落板绝对高度
    nh.param<bool>("use_pad_height", use_pad_height, false);
    nh.param<float>("pad_height", pad_height, 0.01);

    //追踪控制比例参数
    nh.param<float>("kpx_land", kp_land[0], 0.1);
    nh.param<float>("kpy_land", kp_land[1], 0.1);
    nh.param<float>("kpz_land", kp_land[2], 0.1);

    //追踪控制积分参数
    nh.param<float>("kdx_land", kd_land[0], 0.05);
    nh.param<float>("kdy_land", kd_land[1], 0.05);
    nh.param<float>("kdz_land", kd_land[2], 0.05);

     //动态降落控制比例参数
    nh.param<float>("dynamic_kpx_land", dynamic_kp_land[0], 0.1);
    nh.param<float>("dynamic_kpy_land", dynamic_kp_land[1], 0.1);
    nh.param<float>("dynamic_kpz_land", dynamic_kp_land[2], 0.1);

    //动态降落积分参数
    nh.param<float>("dynamic_kdx_land", dynamic_kd_land[0], 0.05);
    nh.param<float>("dynamic_kdy_land", dynamic_kd_land[1], 0.05);
    nh.param<float>("dynamic_kdz_land", dynamic_kd_land[2], 0.05);

    // 初始起飞点
    nh.param<float>("start_point_x", start_point[0], 0.0);
    nh.param<float>("start_point_y", start_point[1], 0.0);
    nh.param<float>("start_point_z", start_point[2], 1.0);

    // 相机安装偏移,规定为:相机在机体系(质心原点)的位置
    nh.param<float>("camera_offset_x", camera_offset[0], 0.0);
    nh.param<float>("camera_offset_y", camera_offset[1], 0.0);
    nh.param<float>("camera_offset_z", camera_offset[2], 0.0);

    //目标运动或静止
    nh.param<bool>("moving_target", moving_target, false);
    nh.param<float>("target_vel_x", target_vel_xy[0], 0.0);
    nh.param<float>("target_vel_y", target_vel_xy[1], 0.0);

    nh.param<double>("average_velocity_time",average_velocity_time,1.0);
    nh.param<double>("average_velocity_time",D_time,0.2);

    nh.param<float>("k",k,1);
    nh.param<float>("k2",k2,1);

    nh.param<float>("delta",delta,1);

    nh.param<float>("dynamic_distance",dynamic_distance,3.0);//切换动态降落的距离
    nh.param<float>("dynamic_height",dynamic_height,3.0);//切换动态降落的距离
    
    nh.param<float>("true_target_vel_x",true_target_vel[0],1.0);//测试用,目标x方向速度
    nh.param<float>("true_target_vel_y",true_target_vel[1],1.0);//测试用,目标x方向速度

    nh.param<int>("window_size",window_size,10);//滑动窗口大小

    // 吊仓追踪参数
    nh.param<float>("gimbal_rate", gimbal_rate, 0.1);
    nh.param<float>("k_gimbal_p", k_gimbal_p, 0.5 );
    nh.param<float>("k_gimbal_i", k_gimbal_i, 0.1 );
    nh.param<float>("k_gimbal_d", k_gimbal_d, 0.5 );
    nh.param<double>("gimbal_window_size", gimbal_window_size, 5);
    // gimbal_control gimbal_control_;


    //打印现实检查参数
    printf_param();

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


    Command_Now.Command_ID = 1;
    Command_Now.source = NODE_NAME;

    if(sim_mode)
    {
        // Waiting for input
        int start_flag = 0;
        while(start_flag == 0)
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<<<< "<< endl;
            cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
            cin >> start_flag;
        }

        while(ros::ok() && _DroneState.mode != "OFFBOARD")
        {
            Command_Now.header.stamp = ros::Time::now();
            Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
            Command_Now.Command_ID = Command_Now.Command_ID + 1;
            Command_Now.source = NODE_NAME;
            Command_Now.Reference_State.yaw_ref = 999;
            command_pub.publish(Command_Now);   
            cout << "Switch to OFFBOARD and arm ..."<<endl;
            ros::Duration(2.0).sleep();
            ros::spinOnce();
        }
    }else
    {
        while(ros::ok() && _DroneState.mode != "OFFBOARD")
        {
            cout << "Waiting for the offboard mode"<<endl;
            ros::Duration(1.0).sleep();
            ros::spinOnce();
        }
    }

    // 起飞
    cout<<"[autonomous_landing]: "<<"Takeoff to predefined position."<<endl;
    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Takeoff to predefined position.");

    while( _DroneState.position[2] < 0.5)
    {      
        Command_Now.header.stamp                        = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
        Command_Now.source                              = NODE_NAME;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = start_point[0];
        Command_Now.Reference_State.position_ref[1]     = start_point[1];
        Command_Now.Reference_State.position_ref[2]     = start_point[2];
        Command_Now.Reference_State.yaw_ref             = 0.0;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();

        ros::spinOnce();
    }

    // 等待
    ros::Duration(3.0).sleep();

    exec_state = EXEC_STATE::WAITING_RESULT;

    while (ros::ok())
    {
        //回调
        ros::spinOnce();


        static int printf_num = 0;
        printf_num++;
        // 此处是为了控制打印频率
        if(printf_num > 20)
        {
            if(exec_state == TRACKING)
            {
                // 正常追踪
                char message_chars[512];
                // char message_chars2[256];
                // sprintf(message_chars, "Tracking the Landing Pad, distance_to_the_pad :   %f [m] .", distance_to_pad);
                snprintf(message_chars, sizeof(message_chars), "Tracking the Landing Pad, distance_to_the_pad : %f [m] . Tracking the Landing Pad, target_velocity : %f [m/s] .", distance_to_pad, velocity.x());
                // sprintf(message_chars2, "Tracking the Landing Pad, target_velocity :   %f [m/s] .", velocity);
                message = message_chars;
                cout << message <<endl;
                pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
            }

            if(sim_mode)
            {
                printf_result();
            }
            
            printf_num = 0;
        }

        // 接收到中断指令，直接降落
        if(mission_cmd.pose.position.x == 99)
        {
            exec_state = LANDING;
        }

        // 接收到hold转降落指令,将设置hold模式为false
        if(mission_cmd.pose.position.x == 88)
        {
            hold_mode = false;
        }

        switch (exec_state)
        {
            // 初始状态，等待视觉检测结果
            case WAITING_RESULT:
            {
                // 发送吊仓指令
                gimbal_control_.send_mount_control_command(gimbal_att_sp_init);

                if(landpad_det.is_detected)
                {
                    exec_state = TRACKING;
                    message = "Get the detection result.";
                    cout << message <<endl;
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                    break;
                }

                // 发送视觉节点启动指令
                vision_switch.data = true;
                vision_switch_pub.publish(vision_switch);
                
                message = "Waiting for the detection result.";
                cout << message <<endl;
                pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                
                ros::Duration(1.0).sleep();
                break;
            }
            // 追踪状态
            case TRACKING:
            {

                if (!is_tracking_initialized) {
                    is_tracking_initialized = true;
                    last_tracking_time = ros::Time::now();
                    last_tracking_position = landpad_det.pos_body_enu_frame;

                    last_tracking_time2 = ros::Time::now();
                    last_tracking_position2 = landpad_det.pos_body_enu_frame;
                }

                // 丢失,进入LOST状态
                if(!landpad_det.is_detected && !hold_mode)
                {
                    exec_state = LOST;
                    message = "Lost the Landing Pad.";
                    cout << message <<endl;
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                    break;
                }   

                distance_to_pad = landpad_det.pos_body_enu_frame.norm();
                if(distance_to_pad < dynamic_distance && abs(landpad_det.pos_body_enu_frame[2] ) < dynamic_height){
                // if(velocity[0]<0.05 && velocity[1]<0.05){
                    exec_state = DYNAMIC_LANDING;
                    message = "切换dynamic_landing";
                    cout<<message<<endl;
                    break;
                }

                // 发送吊仓指令
                gimbal_control_.send_mount_control_command(gimbal_att_sp);
                //【 调用nmpc求解】>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                
                Eigen::Vector2f goal_state ;
                goal_state[0] = current_states[0] + k2 * landpad_det.pos_body_enu_frame[0];
                goal_state[1] = current_states[1] + k2 * landpad_det.pos_body_enu_frame[1];
                
                // nmpc_ctr.set_goal_states(goal_state);
                nmpc_ctr.set_goal_states(landing_pad_states);
                
                Eigen::Vector3f current_states2;
                nmpc_ctr.opti_solution(current_states);

                Eigen::Vector2f nmpc_controls = nmpc_ctr.get_controls();

                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Command_ID = Command_Now.Command_ID + 1;
                Command_Now.source = "nmpc_control";
                Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
                Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::BODY_FRAME;
                Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;
                // xy轴采用mpc
                Command_Now.Reference_State.velocity_ref[0] = nmpc_controls[0];
                Command_Now.Reference_State.velocity_ref[1] = nmpc_controls[1];
                Command_Now.Reference_State.yaw_ref = 0;
                // z轴采用pd
                if(distance_to_pad > dynamic_distance && abs(landpad_det.pos_body_enu_frame[2] ) > dynamic_height ){
                Command_Now.Reference_State.velocity_ref[2] = kp_land[2] * landpad_det.pos_body_enu_frame[2] + kd_land[2] * velocity[2];
                }
                else if(distance_to_pad > dynamic_distance && abs(landpad_det.pos_body_enu_frame[2] ) < dynamic_height ){
                    Command_Now.Reference_State.velocity_ref[2] = 0;
                }

                if (!hold_mode)
                {
                    command_pub.publish(Command_Now);
                }

                // 使用滑动窗口计算平均速度,适用于变速目标>>>>>>>>>>>>>>>>>>>>>>>
                Eigen::Vector3f target_vel;
                Eigen::Vector3f ave_target_vel;
                for(int i=0;i<3;i++){
                    target_vel[i] = _DroneState.velocity[i] + velocity[i];
                }
                num+=1;
                if(buffer.size() == window_size){
                    vel_sum -= buffer.front();
                    buffer.pop_front();
                    buffer.push_back(target_vel);
                    vel_sum += target_vel;
                    ave_target_vel = vel_sum/window_size;
                }
                else if(buffer.size() < window_size){
                    buffer.push_back(target_vel);
                    vel_sum += target_vel;
                    ave_target_vel = vel_sum/buffer.size();
                }


                ROS_INFO("caculated target X: %f, Y: %f", goal_state[0], goal_state[1]);
                ROS_INFO("Drone velocity X: %f, Y: %f", _DroneState.velocity[0], _DroneState.velocity[1]);
                ROS_INFO(" caculated velocity X: %f, Y: %f", velocity[0], velocity[1]);
                ROS_INFO("Target velocity X: %f, Y: %f", target_vel[0], target_vel[1]);
                ROS_INFO("ave_Target velocity X: %f, Y: %f",ave_target_vel[0], ave_target_vel[1]);
                ROS_INFO("-----------------------Tracking---------------------------------");


                break;
            }

            case DYNAMIC_LANDING:
            {
                // 发送吊仓指令
                gimbal_control_.send_mount_control_command(gimbal_att_sp_lock);

                if (!dynamic_tracking_initialized) {
                    dynamic_tracking_initialized = true;
                    last_tracking_time = ros::Time::now();
                    last_tracking_position = landpad_det.pos_body_enu_frame;

                    last_tracking_time2 = ros::Time::now();
                    last_tracking_position2 = landpad_det.pos_body_enu_frame;
                }

                // 丢失,进入LOST状态
                if(!landpad_det.is_detected && !hold_mode)
                {
                    exec_state = LOST;
                    message = "Lost the Landing Pad.";
                    cout << message <<endl;
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                    break;
                }   

                // 抵达上锁点,进入LANDING
                distance_to_pad = landpad_det.pos_body_enu_frame.norm();
                //　达到降落距离，上锁降落
                if(distance_to_pad < arm_distance_to_pad)
                {
                    exec_state = LANDING;
                    message = "Catched the Landing Pad.";
                    cout << message <<endl;
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                    break;
                }
                //　达到最低高度，上锁降落
                else if(abs(landpad_det.pos_body_enu_frame[2]) < arm_height_to_ground)
                {
                    exec_state = LANDING;
                    message = "Reach the lowest height.";
                    cout << message <<endl;
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                    break;
                }

                // 机体系速度控制
                Command_Now.header.stamp = ros::Time::now();
                Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                Command_Now.source = NODE_NAME;
                Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
                Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
                Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_VEL;   //xy velocity z position
                
               
                // for (int i=0; i<3; i++)
                // {
                //     // Command_Now.Reference_State.velocity_ref[i] = kp_land[i] * landpad_det.pos_body_enu_frame[i];
                //     Command_Now.Reference_State.velocity_ref[i] = dynamic_kp_land[i] * landpad_det.pos_body_enu_frame[i] + dynamic_kd_land[i] * velocity2[i];
                // }


                // 使用滑动窗口计算平均速度,适用于变速目标>>>>>>>>>>>>>>>>>>>>>>>
                Eigen::Vector3f target_vel;
                Eigen::Vector3f ave_target_vel;
                for(int i=0;i<3;i++){
                    target_vel[i] = _DroneState.velocity[i] + velocity[i];
                }
                num+=1;
                if(buffer.size() == window_size){
                    vel_sum -= buffer.front();
                    buffer.pop_front();
                    buffer.push_back(target_vel);
                    vel_sum += target_vel;
                    ave_target_vel = vel_sum/window_size;
                }
                else if(buffer.size() < window_size){
                    buffer.push_back(target_vel);
                    vel_sum += target_vel;
                    ave_target_vel = vel_sum/buffer.size();
                }


                ROS_INFO("Drone velocity X: %f, Y: %f", _DroneState.velocity[0], _DroneState.velocity[1]);
                ROS_INFO("Target velocity X: %f, Y: %f", target_vel[0], target_vel[1]);
                ROS_INFO("ave_Target velocity X: %f, Y: %f",ave_target_vel[0], ave_target_vel[1]);
                ROS_INFO("---------------------Dynamic landing-----------------------------------");

                 //【 调用nmpc求解】>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                
                Eigen::Vector2f goal_state ;
                goal_state[0] = current_states[0] + k2 * landpad_det.pos_body_enu_frame[0];
                goal_state[1] = current_states[1] + k2 * landpad_det.pos_body_enu_frame[1];
                
                // nmpc_ctr.set_goal_states(goal_state);
                nmpc_ctr.set_goal_states(landing_pad_states);
                
                Eigen::Vector3f current_states2;
                nmpc_ctr.opti_solution(current_states);

                Eigen::Vector2f nmpc_controls = nmpc_ctr.get_controls();

                if(moving_target)//这里简单加上目标速度效果并不好
                {
                    // Command_Now.Reference_State.velocity_ref[0] += 0.3 * target_vel_xy[0];
                    // Command_Now.Reference_State.velocity_ref[1] += 0.3 * target_vel_xy[1];
                    // Command_Now.Reference_State.velocity_ref[0] += k * ave_target_vel[0];
                    // Command_Now.Reference_State.velocity_ref[1] += k * ave_target_vel[1];

                    

                    Command_Now.Reference_State.velocity_ref[0] = k * ave_target_vel[0] + dynamic_kp_land[0] * (roi_point[0] - mav_pos_[0]);
                    Command_Now.Reference_State.velocity_ref[1] = k * ave_target_vel[1] + dynamic_kp_land[0] * (roi_point[1] - mav_pos_[1]);

                    Command_Now.Reference_State.velocity_ref[0] = (1-delta) * Command_Now.Reference_State.velocity_ref[0] + delta * nmpc_controls[0];
                    Command_Now.Reference_State.velocity_ref[1] = (1-delta) * Command_Now.Reference_State.velocity_ref[1] + delta * nmpc_controls[1];

                    delta = delta * 0.8;

                    if(abs(landpad_det.pos_body_enu_frame[2] ) > dynamic_height/2){
                        Command_Now.Reference_State.velocity_ref[2] =  kp_land[2] * landpad_det.pos_body_enu_frame[2] + kd_land[2] * velocity[2];
                    }else{
                        Command_Now.Reference_State.velocity_ref[2] = 0.0;
                    }
                }

                if(!moving_target){
                    Command_Now.Reference_State.velocity_ref[0] += k * true_target_vel[0];
                    Command_Now.Reference_State.velocity_ref[1] += k * true_target_vel[1];
                }

                Command_Now.Reference_State.yaw_ref             = 0.0;
                //Publish

                if (!hold_mode)
                {
                    command_pub.publish(Command_Now);
                }

                break;
            }

            case LOST:
            {

                // 发送吊仓指令
                gimbal_control_.send_mount_control_command(gimbal_att_sp_lock);

                static int lost_time = 0;
                lost_time ++ ;
                
                // 重新获得信息,进入TRACKING
                if(landpad_det.is_detected)
                {
                    is_tracking_initialized = false;
                    dynamic_tracking_initialized = false;
                    exec_state = TRACKING;
                    lost_time = 0;
                    message = "Regain the Landing Pad.";
                    cout << message <<endl;
                    pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                    break;
                }   
                
                // 首先是悬停等待 尝试得到图像, 如果仍然获得不到图像 则原地上升
                if(lost_time < 10.0)
                {
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                    Command_Now.source = NODE_NAME;
                    Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;

                    ros::Duration(0.4).sleep();
                }else
                {
                    Command_Now.header.stamp                        = ros::Time::now();
                    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
                    Command_Now.Command_ID                          = Command_Now.Command_ID + 1;
                    Command_Now.source                              = NODE_NAME;
                    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_VEL;
                    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::BODY_FRAME;
                    Command_Now.Reference_State.velocity_ref[0]     = 0.0;
                    Command_Now.Reference_State.velocity_ref[1]     = 0.0;
                    Command_Now.Reference_State.velocity_ref[2]     = 0.1;
                    Command_Now.Reference_State.yaw_ref             = 0;

                    // 如果上升超过原始高度，则认为任务失败，则直接降落
                    if(_DroneState.position[2] >= start_point[2])
                    {
                        exec_state = LANDING;
                        lost_time = 0;
                        message = "Mission failed, landing... ";
                        cout << message <<endl;
                        pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, message);
                        break;
                    }
                }
                command_pub.publish(Command_Now);
                break;
            }
            case LANDING:
            {
                if(sim_mode)    //？？
                {
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                    Command_Now.source = NODE_NAME;
                    Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm; // 新飞控不支持直接上锁,会变成返航模式
                    // Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
                    command_pub.publish(Command_Now);
                }else
                {
                    Command_Now.header.stamp = ros::Time::now();
                    Command_Now.Command_ID   = Command_Now.Command_ID + 1;
                    Command_Now.source = NODE_NAME;
                    Command_Now.Mode = prometheus_msgs::ControlCommand::Land;
                    command_pub.publish(Command_Now);
                }


                // ros::Duration(1.0).sleep();

                break;
            }
        }
      
        rate.sleep();

    }

    return 0;

}

void printf_result()
{

    cout << ">>>>>>>>>>>>>>>>>>>>>>Autonomous Landing Mission<<<<<<<<<<<<<<<<<<<"<< endl;

    switch (exec_state)
    {
        case WAITING_RESULT:
            cout << "exec_state: WAITING_RESULT" <<endl;
            break;
        case TRACKING:
            cout << "exec_state: TRACKING" <<endl;
            break;
        case LOST:
            cout << "exec_state: LOST" <<endl;
            break;
        case LANDING:
            cout << "exec_state: LANDING" <<endl;
            break;
    } 

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if(landpad_det.is_detected)
    {
        cout << "is_detected: ture" <<endl;
    }else
    {
        cout << "is_detected: false" <<endl;
    }
    
    cout << "Target_pos (body): " << landpad_det.pos_body_frame[0] << " [m] "<< landpad_det.pos_body_frame[1] << " [m] "<< landpad_det.pos_body_frame[2] << " [m] "<<endl;

    cout << "Target_pos (body_enu): " << landpad_det.pos_body_enu_frame[0] << " [m] "<< landpad_det.pos_body_enu_frame[1] << " [m] "<< landpad_det.pos_body_enu_frame[2] << " [m] "<<endl;

    cout << "Ground_truth(pos):  " << GroundTruth.pose.pose.position.x << " [m] "<< GroundTruth.pose.pose.position.y << " [m] "<< GroundTruth.pose.pose.position.z << " [m] "<<endl;
    cout << "Detection_ENU(pos): " << landpad_det.pos_enu_frame[0] << " [m] "<< landpad_det.pos_enu_frame[1] << " [m] "<< landpad_det.pos_enu_frame[2] << " [m] "<<endl;
    cout << "Detection_ENU(yaw): " << landpad_det.att_enu_frame[2]/3.1415926 *180 << " [deg] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Land Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "vel_cmd: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[1] << " [m/s] "<< Command_Now.Reference_State.velocity_ref[2] << " [m/s] "<<endl;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "hold_mode : "<< hold_mode << endl;
    cout << "sim_mode : "<< sim_mode << endl;
    cout << "use_pad_height : "<< use_pad_height << endl;
    

    cout << "arm_distance_to_pad : "<< arm_distance_to_pad << endl;
    cout << "arm_height_to_ground : "<< arm_height_to_ground << endl;
    cout << "kpx_land : "<< kp_land[0] << endl;
    cout << "kpy_land : "<< kp_land[1] << endl;
    cout << "kpz_land : "<< kp_land[2] << endl;
    cout << "start_point_x : "<< start_point[0] << endl;
    cout << "start_point_y : "<< start_point[1] << endl;
    cout << "start_point_z : "<< start_point[2] << endl;
    cout << "camera_offset_x : "<< camera_offset[0] << endl;
    cout << "camera_offset_y : "<< camera_offset[1] << endl;
    cout << "camera_offset_z : "<< camera_offset[2] << endl;
    

    cout << "moving_target : "<< moving_target << endl;
    cout << "target_vel_x : "<< target_vel_xy[0] << endl;
    cout << "target_vel_y : "<< target_vel_xy[1] << endl;

    cout << "gimbal_att    : " << gimbal_att_deg[0] << " [deg] "<< gimbal_att_deg[1] << " [deg] "<< gimbal_att_deg[2] << " [deg] "<<endl;

}
