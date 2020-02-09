/***************************************************************************************************************************
 * autonomous_landing.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2020.1.12
 *
 * 说明: 目标追踪示例程序
 *      1. 订阅目标位置(来自视觉的ros节点)
 *      2. 追踪算法及追踪策略
 *      3. 发布上层控制指令 (prometheus_msgs::ControlCommand)
***************************************************************************************************************************/
//ROS 头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>

//topic 头文件
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include <prometheus_msgs/ControlCommand.h>
#include <geometry_msgs/Point.h>


using namespace std;
using namespace Eigen;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//---------------------------------------Drone---------------------------------------------
prometheus_msgs::DroneState _DroneState;   
Eigen::Vector3f drone_pos;
//---------------------------------------Vision---------------------------------------------
prometheus_msgs::DetectionInfo Detection_raw;          //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
Eigen::Vector3f pos_body_frame;
Eigen::Vector3f pos_des_prev;
float kpx_land,kpy_land,kpz_land;                                                 //控制参数 - 比例参数

bool is_detected = false;                                          // 是否检测到目标标志
int num_count_vision_lost = 0;                                                      //视觉丢失计数器
int num_count_vision_regain = 0;                                                      //视觉丢失计数器
int Thres_vision = 0;                                                          //视觉丢失计数器阈值
//---------------------------------------Track---------------------------------------------
float distance_to_setpoint;
float distance_thres;
float landing_pad_height;
//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_param();                                                                 //打印各项参数以供检查
void printf_result();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void vision_cb(const prometheus_msgs::DetectionInfo::ConstPtr &msg)
{
    Detection_raw = *msg;

    pos_body_frame[0] = -Detection_raw.position[1];
    pos_body_frame[1] = -Detection_raw.position[0];
    pos_body_frame[2] = -Detection_raw.position[2]; 

    
    if(Detection_raw.detected)
    {
        num_count_vision_regain++;
        num_count_vision_lost = 0;
    }else
    {
        num_count_vision_regain = 0;
        num_count_vision_lost++;
    }

    // 当连续一段时间无法检测到目标时，认定目标丢失
    if(num_count_vision_lost > Thres_vision)
    {
        is_detected = false;
    }

    // 当连续一段时间检测到目标时，认定目标得到
    if(num_count_vision_regain > Thres_vision)
    {
        is_detected = true;
    }
}

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    drone_pos[0] = _DroneState.position[0];
    drone_pos[1] = _DroneState.position[1];
    drone_pos[2] = _DroneState.position[2];
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonomous_landing");
    ros::NodeHandle nh("~");

    //节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(20.0);

    //【订阅】降落板与无人机的相对位置及相对偏航角  单位：米   单位：弧度
    // 来自视觉节点 方向定义：[机体系下：前方x为正，右方y为正，下方z为正]
    ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/target", 10, vision_cb);

    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    //【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //视觉丢失次数阈值
    nh.param<int>("Thres_vision", Thres_vision, 10);

    //追踪距离阈值
    nh.param<float>("distance_thres", distance_thres, 0.2);

    //降落板高度
    nh.param<float>("landing_pad_height", landing_pad_height, 0.0);

    //追踪的前后间隔
    nh.param<float>("kpx_land", kpx_land, 0.1);
    nh.param<float>("kpy_land", kpy_land, 0.1);
    nh.param<float>("kpz_land", kpz_land, 0.1);

    //打印现实检查参数
    printf_param();

    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    // 先读取一些飞控的数据
    for(int i=0;i<10;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    pos_des_prev[0] = drone_pos[0];
    pos_des_prev[1] = drone_pos[1];
    pos_des_prev[2] = drone_pos[2];

    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID                          = 0;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 0;
    Command_Now.Reference_State.position_ref[1]     = 0;
    Command_Now.Reference_State.position_ref[2]     = 0;
    Command_Now.Reference_State.velocity_ref[0]     = 0;
    Command_Now.Reference_State.velocity_ref[1]     = 0;
    Command_Now.Reference_State.velocity_ref[2]     = 0;
    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;
    Command_Now.Reference_State.yaw_ref             = 0;

    // 起飞
    cout<<"[autonomous_landing]: "<<"Takeoff to predefined position."<<endl;
    Command_Now.header.stamp                    = ros::Time::now();
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
    Command_Now.Command_ID                          = 1;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 1;
    Command_Now.Reference_State.position_ref[1]     = 1;
    Command_Now.Reference_State.position_ref[2]     = 2.5;
    Command_Now.Reference_State.yaw_ref             = 0;

    //command_pub.publish(Command_Now);

    //sleep(8.0);

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Command_ID                      = Command_Now.Command_ID + 1;

        printf_result();

        //判断是否满足降落条件
        distance_to_setpoint = pos_body_frame.norm();
        if(distance_to_setpoint < distance_thres)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm;
            cout <<"[autonomous_landing]: Catched the Landing Pad, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
        }

        if(!is_detected)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
            pos_des_prev[0] = drone_pos[0];
            pos_des_prev[1] = drone_pos[1];
            pos_des_prev[2] = drone_pos[2];
            cout <<"[autonomous_landing]: Lost the Landing Pad. "<< endl;
        }else if(abs(pos_body_frame[2]) < 0.3)
        {
            cout <<"[autonomous_landing]: Reach the lowest height. "<< endl;
            Command_Now.Mode = prometheus_msgs::ControlCommand::Disarm;
        }else
        {
            cout <<"[autonomous_landing]: Tracking the Landing Pad, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
            Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
            Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
            Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;   //xy velocity z position

            Eigen::Vector3f vel_command;
            vel_command[0] = kpx_land * pos_body_frame[0];
            vel_command[1] = kpy_land * pos_body_frame[1];
            vel_command[2] = kpz_land * pos_body_frame[2];

            for (int i=0; i<3; i++)
            {
                Command_Now.Reference_State.position_ref[i] = pos_des_prev[i] + vel_command[i]* 0.05;
            }
            Command_Now.Reference_State.yaw_ref             = 0.0;
            
            for (int i=0; i<3; i++)
            {
                pos_des_prev[i] = Command_Now.Reference_State.position_ref[i];
            }

        }

        //Publish
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Command_ID   = Command_Now.Command_ID + 1;
        command_pub.publish(Command_Now);

        rate.sleep();

    }

    return 0;

}


void printf_result()
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(4);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Autonomous Landing<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if(is_detected)
    {
        cout << "is_detected: ture" <<endl;
    }else
    {
        cout << "is_detected: false" <<endl;
    }
    
    cout << "Detection_raw: " << Detection_raw.position[0] << " [m] "<< Detection_raw.position[1] << " [m] "<< Detection_raw.position[2] << " [m] "<<endl;
    cout << "Detection_raw: " << Detection_raw.attitude[2]/3.1415926 *180 << " [du] "<<endl;
    
    cout << "pos_body_frame: " << pos_body_frame[0] << " [m] "<< pos_body_frame[1] << " [m] "<< pos_body_frame[2] << " [m] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Land Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "pos_des: " << Command_Now.Reference_State.position_ref[0] << " [m] "<< Command_Now.Reference_State.position_ref[1] << " [m] "<< Command_Now.Reference_State.position_ref[2] << " [m] "<<endl;
}
void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Thres_vision : "<< Thres_vision << endl;
    cout << "distance_thres : "<< distance_thres << endl;
    cout << "landing_pad_height : "<< landing_pad_height << endl;
    cout << "kpx_land : "<< kpx_land << endl;
    cout << "kpy_land : "<< kpy_land << endl;
    cout << "kpz_land : "<< kpz_land << endl;
}
