#include <ros/ros.h>
#include "nmpc_ctr.h"
#include <geometry_msgs/PoseStamped.h> //获取rviz目标点数据
#include <nav_msgs/Odometry.h> //获取里程计数据
#include <geometry_msgs/Twist.h> //发布cmd_vel
#include <nav_msgs/Path.h> //发布预测轨迹点
#include <visualization_msgs/Marker.h> //以箭头形式在rviz中可视化目标点
#include <tf/tf.h>

#include "prometheus_msgs/ControlCommand.h"
#include "std_msgs/Header.h"
#include <cmath> // 引入 std::round

// 接收rviz2D目标点
class GoalStatesListener
{
public:
    Eigen::Matrix<float,2,1> m_goal_states;
    bool received_goal = false;

public:
    GoalStatesListener();
    ~GoalStatesListener();
    void callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

GoalStatesListener::GoalStatesListener()
{

}

GoalStatesListener::~GoalStatesListener()
{

}

void GoalStatesListener::callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double x=msg->pose.position.x;
    double y=msg->pose.position.y;
    //四元数转欧拉角
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    //输出
    // m_goal_states<<x,y,yaw;
    m_goal_states<<x,y;
    received_goal = true;
}




//通过订阅无人机当前里程计话题获取小车实时状态
class CurrentStatesListener
{
public:
    Eigen::Matrix<float,2,1> m_current_states;

public:
    CurrentStatesListener();
    ~CurrentStatesListener();
    void callback(const nav_msgs::Odometry::ConstPtr& msg);
};

CurrentStatesListener::CurrentStatesListener()
{

}

CurrentStatesListener::~CurrentStatesListener()
{

}

void CurrentStatesListener::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double x, y, z;
    double roll, pitch, yaw;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;
    //四元数转欧拉角
    tf::Quaternion quat;                                     //定义一个四元数
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat); //取出方向存储于四元数
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    // m_current_states<<x,y,yaw;
    m_current_states<<x,y;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"uav_control");
    ros::NodeHandle n;

    //创建NMPC控制器
    int predict_step = 50;
    float sample_time = 0.1;
    NMPC nmpc_ctr(predict_step, sample_time);

    //创建监听对象
    GoalStatesListener mlistener1;

    ros::Subscriber sub1 = n.subscribe("/move_base_simple/goal",100,&GoalStatesListener::callback,&mlistener1);
    //创建监听对象
    CurrentStatesListener mlistener2;

    ros::Subscriber sub2 = n.subscribe("/prometheus/drone_odom",100,&CurrentStatesListener::callback,&mlistener2);

    //创建控制量发布对象
    ros::Publisher controls_pub =n.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command",100);

    //创建预测轨迹发布对象
    ros::Publisher path_pub =n.advertise<nav_msgs::Path>("/robot1/predict_trajectory",100);

    //创建rviz目标点可视化发布对象
    ros::Publisher marker_pub =n.advertise<visualization_msgs::Marker>("/robot1/goal_marker",100);


    // 设置循环的频率
    ros::Rate loop_rate(10);

    

    int command_id = 4; // Start counting from 4

    while(ros::ok() && !mlistener1.received_goal){
        ros::spinOnce();
        loop_rate.sleep();
    }
    while(ros::ok()){

        ros::spinOnce();

        ros::Time current_time = ros::Time::now();

        //获取目标点数据
        nmpc_ctr.set_goal_states(mlistener1.m_goal_states);

        //发布目标点数据，在rviz中可视化
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = current_time;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        
        marker.color.g = 1.0f; // 点为红色
        marker.color.a = 1.0;
        geometry_msgs::Point p1, p2;
        p1.x = mlistener1.m_goal_states[0];
        p1.y = mlistener1.m_goal_states[1];
        p1.z = 0;
        // p2.x = mlistener1.m_goal_states[0]+0.2*std::cos(mlistener1.m_goal_states[2]);
        // p2.y = mlistener1.m_goal_states[1]+0.2*std::sin(mlistener1.m_goal_states[2]);
        p2.x = mlistener1.m_goal_states[0];
        p2.y = mlistener1.m_goal_states[1];
        p2.z = 0;
        marker.points.push_back(p1) ;
        marker.points.push_back(p2) ;
        marker_pub.publish(marker) ;

        //调用nmpc求解
        nmpc_ctr.opti_solution(mlistener2.m_current_states);
        //获取控制量
        Eigen::Vector2f nmpc_controls = nmpc_ctr.get_controls();
        //获取预测轨迹点
        std::vector<Eigen::Matrix<float,2,1>> predict_trajectory;
        predict_trajectory = nmpc_ctr.get_predict_trajectory();
        int len_predict_trajectory = predict_trajectory.size();

         //发布预测轨迹点
        nav_msgs::Path path;
        path.header.stamp=current_time;
        path.header.frame_id="world";
        geometry_msgs::PoseStamped this_pose_stamped;

        for(int k=0;k<len_predict_trajectory;k++)
        {
            this_pose_stamped.pose.position.x = predict_trajectory.at(k)[0];
            this_pose_stamped.pose.position.y = predict_trajectory.at(k)[1];
            // geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(predict_trajectory.at(k)[2]);
            geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(0);
            this_pose_stamped.pose.orientation.x = goal_quat.x;
            this_pose_stamped.pose.orientation.y = goal_quat.y;
            this_pose_stamped.pose.orientation.z = goal_quat.z;
            this_pose_stamped.pose.orientation.w = goal_quat.w;
            this_pose_stamped.header.stamp=current_time;
            this_pose_stamped.header.frame_id="base_link";
            path.poses.push_back(this_pose_stamped);
        }
        path_pub.publish(path);

        // 发布控制量
        prometheus_msgs::ControlCommand command;
        // 设置消息头
        // command.header.seq = 0;
        command.header.stamp = ros::Time::now();
        command.header.frame_id = "";

        command.Command_ID = command_id++;
        command.source = "mpc_control";
        command.Mode = 4;
        float vx = nmpc_controls[0];
        float vy = nmpc_controls[1];
        // float vyaw = nmpc_controls[2];
        // 设置Reference_State
        command.Reference_State.header.seq = 0;
        command.Reference_State.header.stamp = ros::Time(0, 0);
        command.Reference_State.header.frame_id = "";
        command.Reference_State.Move_mode = 2;
        command.Reference_State.Move_frame = 1;
        command.Reference_State.time_from_start = 0.0;
        command.Reference_State.position_ref = {0.0, 0.0, 0.0};
        command.Reference_State.velocity_ref = {vx, vy, 0.0};
        command.Reference_State.acceleration_ref = {0.0, 0.0, 0.0};
        command.Reference_State.latitude = 0.0;
        command.Reference_State.longitude = 0.0;
        command.Reference_State.altitude = 0.0;
        command.Reference_State.Yaw_Rate_Mode = false;
        command.Reference_State.yaw_ref = 0;
        command.Reference_State.yaw_rate_ref = 0.0;

        // 发布消息
        controls_pub.publish(command);

        //休眠
        loop_rate.sleep();
    }

    return 0;
}