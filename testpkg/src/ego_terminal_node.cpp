#include <ros/ros.h>
#include <iostream>

#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <prometheus_msgs/ControlCommand.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>

#include "quadrotor_msgs/GoalSet.h"
#include "quadrotor_msgs/PositionCommand.h"

#define VEL_XY_STEP_SIZE 0.1
#define VEL_Z_STEP_SIZE 0.1
#define YAW_STEP_SIZE 0.08
#define TRA_WINDOW 2000
#define NODE_NAME "ego_terminal"

using namespace std;

ros::Publisher cmd_pub;
prometheus_msgs::ControlCommand Command_to_pub;

ros::Publisher ego_goal_pub;
quadrotor_msgs::GoalSet ego_goal_to_pub;

ros::Publisher ego_stop_pub;
std_msgs::Empty ego_stop_to_pub;

bool ego_flag = false;
ros::Subscriber ego_cmd_sub;
quadrotor_msgs::PositionCommand ego_cmd;

tf::TransformListener* tfListener;

void mainloop();
void generate_com(int Move_mode, float state_desired[4]);
void ego_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);

template <typename Scalar_t>
Scalar_t normalize_angle(Scalar_t a) {
    int cnt = 0;
    while (true) {
        cnt++;

        if (a < -M_PI) {
            a += M_PI * 2.0;
        } else if (a > M_PI) {
            a -= M_PI * 2.0;
        }

        if (-M_PI <= a && a <= M_PI) {
            break;
        };

        assert(cnt < 10 && "[uav_utils/geometry_msgs] INVALID INPUT ANGLE");
    }

    return a;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_terminal");
  ros::NodeHandle nh;
  tfListener = new tf::TransformListener();


  //　【发布】　prometheus 控制指令
  cmd_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

  //　【发布】　ego-planner 目标点
  ego_goal_pub = nh.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 1);

  //　【发布】　ego-planner 手动停止
  ego_stop_pub = nh.advertise<std_msgs::Empty>("/mandatory_stop_to_planner", 1);

  //　【订阅】　ego-planner 控制指令
  ego_cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/drone_0_planning/pos_cmd", 1, ego_cmd_cb);

  // 初始化命令 - Idle模式 电机怠速旋转 等待来自上层的控制指令
  Command_to_pub.Mode                                = prometheus_msgs::ControlCommand::Idle;
  Command_to_pub.Command_ID                          = 0;
  Command_to_pub.source = NODE_NAME;
  Command_to_pub.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
  Command_to_pub.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
  Command_to_pub.Reference_State.position_ref[0]     = 0;
  Command_to_pub.Reference_State.position_ref[1]     = 0;
  Command_to_pub.Reference_State.position_ref[2]     = 0;
  Command_to_pub.Reference_State.velocity_ref[0]     = 0;
  Command_to_pub.Reference_State.velocity_ref[1]     = 0;
  Command_to_pub.Reference_State.velocity_ref[2]     = 0;
  Command_to_pub.Reference_State.acceleration_ref[0] = 0;
  Command_to_pub.Reference_State.acceleration_ref[1] = 0;
  Command_to_pub.Reference_State.acceleration_ref[2] = 0;
  Command_to_pub.Reference_State.yaw_ref             = 0;

  //固定的浮点显示
  cout.setf(ios::fixed);
  //setprecision(n) 设显示小数精度为n位
  cout<<setprecision(2);
  //左对齐
  cout.setf(ios::left);
  // 强制显示小数点
  cout.setf(ios::showpoint);
  // 强制显示符号
  //cout.setf(ios::showpos);

  // 默认终端输入控制
  cout << ">>>>>>>>>>>>>>>> Welcome to use Ego Terminal Control <<<<<<<<<<<<<<<<"<< endl;

  mainloop();

  return 0;
}

void mainloop()
{
  int Control_Mode = 0;
  int Move_mode = 0;
  int Move_frame = 0;
  int Trjectory_mode = 0;
  float state_desired[4];

  int command_flag = 0;
  float goal[3];
  int last_cmd_seq = -1;

  while(ros::ok())
  {
    
    // Waiting for input
    cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus Terminal Control <<<<<<<<<<<<<<<<"<< endl;
    cout << "Please choose the Command.Mode: 0 for Idle, 1 for Takeoff, 2 for Hold, 3 for Land, 4 for Move, 5 for Disarm, 6 for User_Mode1, 7 for User_Mode2"<<endl;
    cout << "Input 999 to switch to offboard mode and arm the drone (ONLY for simulation, please use RC in experiment!!!)"<<endl;
    cin  >> Control_Mode;

    if (Control_Mode == prometheus_msgs::ControlCommand::Move)
    {
      cout << "Please choose the Command.Reference_State.Move_mode: 0 for XYZ_POS, 1 for XY_POS_Z_VEL, 2 for XY_VEL_Z_POS, 3 for XYZ_VEL"<<endl;
      cin >> Move_mode;

      cout << "Please choose the Command.Reference_State.Move_frame: 0 for ENU_FRAME, 1 for BODY_FRAME"<<endl;
      cin >> Move_frame; 
      cout << "Please input the reference state [x y z yaw]: "<< endl;
      cout << "setpoint_t[0] --- x [m] : "<< endl;
      cin >> state_desired[0];
      cout << "setpoint_t[1] --- y [m] : "<< endl;
      cin >> state_desired[1];
      cout << "setpoint_t[2] --- z [m] : "<< endl;
      cin >> state_desired[2];
      cout << "setpoint_t[3] --- yaw [du] : "<< endl;
      cin >> state_desired[3];
    }
    else if (Control_Mode == prometheus_msgs::ControlCommand::User_Mode1)
    {
      cout << "Please set the goal, 0 for BODY, 1 for ENU"<<endl;
      cin >> command_flag;
      if (command_flag == 0)
      {
        // input the point in body frame
        float x_body, y_body, z_body;
        cout << "Please input the goal point [x y z]: "<< endl;
        cout << "goal[0] --- x [m] : "<< endl;
        cin >> x_body;
        cout << "goal[1] --- y [m] : "<< endl;
        cin >> y_body;
        cout << "goal[2] --- z [m] : "<< endl;
        cin >> z_body;

        geometry_msgs::PointStamped p_body;
        p_body.header.frame_id = "base_link";
        p_body.header.stamp = ros::Time::now();
        p_body.point.x = x_body;
        p_body.point.y = y_body;
        p_body.point.z = z_body;
        try
        {
          if (tfListener->waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(1.0)))
          {
            geometry_msgs::PointStamped p_map;
            tfListener->transformPoint("map", p_body, p_map);
            cout << "goal in map frame: " << p_map.point.x << " [m] "<< p_map.point.y << " [m] "<< p_map.point.z << " [m] "<< endl;
            goal[0] = p_map.point.x;
            goal[1] = p_map.point.y;
            goal[2] = p_map.point.z;
            ego_flag = true;
          }
        }
        catch (tf::TransformException &ex)
        {
          ROS_WARN("%s", ex.what());
          ros::Duration(1.0).sleep();
        }
      }
      else
      {
        cout << "Please input the goal point [x y z]: "<< endl;
        cout << "goal[0] --- x [m] : "<< endl;
        cin >> goal[0];
        cout << "goal[1] --- y [m] : "<< endl;
        cin >> goal[1];
        cout << "goal[2] --- z [m] : "<< endl;
        cin >> goal[2];
        ego_flag = true;
      }
    }
    else if (Control_Mode == 999)
    {
      Command_to_pub.header.stamp = ros::Time::now();
      Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
      Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
      Command_to_pub.source = NODE_NAME;
      Command_to_pub.Reference_State.yaw_ref = 999;
      cmd_pub.publish(Command_to_pub);
      Command_to_pub.Reference_State.yaw_ref = 0.0;
    }

    int count = 0;
    switch (Control_Mode)
    {
      case prometheus_msgs::ControlCommand::Idle:
        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Idle;
        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
        Command_to_pub.source = NODE_NAME;
        cmd_pub.publish(Command_to_pub);
        break;

      case prometheus_msgs::ControlCommand::Takeoff:
        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Takeoff;
        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
        Command_to_pub.source = NODE_NAME;
        cmd_pub.publish(Command_to_pub);
        break;

      case prometheus_msgs::ControlCommand::Hold:
        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Hold;
        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
        Command_to_pub.source = NODE_NAME;
        cmd_pub.publish(Command_to_pub);
        break;
    
      case prometheus_msgs::ControlCommand::Land:
        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Land;
        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
        Command_to_pub.source = NODE_NAME;
        cmd_pub.publish(Command_to_pub);
        break;

      case prometheus_msgs::ControlCommand::Move:
        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
        Command_to_pub.source = NODE_NAME;
        Command_to_pub.Reference_State.Move_mode  = Move_mode;
        Command_to_pub.Reference_State.Move_frame = Move_frame;
        Command_to_pub.Reference_State.time_from_start = -1;
        generate_com(Move_mode, state_desired);

        cmd_pub.publish(Command_to_pub);
        break;
            
      case prometheus_msgs::ControlCommand::Disarm:
        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.Mode = prometheus_msgs::ControlCommand::Disarm;
        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
        Command_to_pub.source = NODE_NAME;
        cmd_pub.publish(Command_to_pub);
        break;

      case prometheus_msgs::ControlCommand::User_Mode1:
        ego_goal_to_pub.drone_id = 0;
        ego_goal_to_pub.goal[0] = goal[0];
        ego_goal_to_pub.goal[1] = goal[1];
        ego_goal_to_pub.goal[2] = goal[2];
        ego_goal_pub.publish(ego_goal_to_pub);
        last_cmd_seq = -1;

        ros::Duration(0.5).sleep();
        while (ego_flag == true)
        {
          ros::spinOnce();
          if (ego_cmd.header.seq == last_cmd_seq) // trajectory finish, no cmd from traj_server
          {
            cout << "Ego trajectory finish!"<<  last_cmd_seq<< endl;
            ego_flag = false;
          }

          last_cmd_seq = ego_cmd.header.seq;
          if (count >= 300)
          {
            cout << "Executing ego trajectory, seq " << last_cmd_seq << endl;
            count = 0;
          }
          count++;
          // dont be higher than the rate of traj_server (100Hz)
          ros::Duration(0.05).sleep();
        }
        break;
          
      case prometheus_msgs::ControlCommand::User_Mode2:
        Command_to_pub.header.stamp = ros::Time::now();
        Command_to_pub.Mode = prometheus_msgs::ControlCommand::User_Mode2;
        Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
        Command_to_pub.source = NODE_NAME;
        cmd_pub.publish(Command_to_pub);
        break;
    }
    sleep(1.0);
    cout << "....................................................." <<endl;
  }
}

void ego_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
  // cout << "ego traj cmd received " << msg->header.seq << endl;
  if (ego_flag == false)
  {
    cout << "traj cmd not used" << endl;

    return;
  }
  
  ego_cmd = *msg;

  Command_to_pub.header.stamp = ros::Time::now();
  Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
  Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
  Command_to_pub.source = NODE_NAME;
  Command_to_pub.Reference_State.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;

  Command_to_pub.Reference_State.position_ref[0]     = ego_cmd.position.x;
  Command_to_pub.Reference_State.position_ref[1]     = ego_cmd.position.y;
  Command_to_pub.Reference_State.position_ref[2]     = ego_cmd.position.z;
  Command_to_pub.Reference_State.velocity_ref[0]     = ego_cmd.velocity.x;
  Command_to_pub.Reference_State.velocity_ref[1]     = ego_cmd.velocity.y;
  Command_to_pub.Reference_State.velocity_ref[2]     = ego_cmd.velocity.z;
  Command_to_pub.Reference_State.acceleration_ref[0]     = ego_cmd.acceleration.x;
  Command_to_pub.Reference_State.acceleration_ref[1]     = ego_cmd.acceleration.y;
  Command_to_pub.Reference_State.acceleration_ref[2]     = ego_cmd.acceleration.z;
  Command_to_pub.Reference_State.yaw_ref = normalize_angle(ego_cmd.yaw);

  cmd_pub.publish(Command_to_pub);
}

void generate_com(int Move_mode, float state_desired[4])
{
    //# Move_mode 2-bit value:
    //# 0 for position, 1 for vel, 1st for xy, 2nd for z.
    //#                   xy position     xy velocity
    //# z position       	0b00(0)       0b10(2)
    //# z velocity		0b01(1)       0b11(3)

    if(Move_mode == prometheus_msgs::PositionReference::XYZ_ACC)
    {
        cout << "ACC control not support yet." <<endl;
    }

    if((Move_mode & 0b10) == 0) //xy channel
    {
        Command_to_pub.Reference_State.position_ref[0] = state_desired[0];
        Command_to_pub.Reference_State.position_ref[1] = state_desired[1];
        Command_to_pub.Reference_State.velocity_ref[0] = 0;
        Command_to_pub.Reference_State.velocity_ref[1] = 0;
    }
    else
    {
        Command_to_pub.Reference_State.position_ref[0] = 0;
        Command_to_pub.Reference_State.position_ref[1] = 0;
        Command_to_pub.Reference_State.velocity_ref[0] = state_desired[0];
        Command_to_pub.Reference_State.velocity_ref[1] = state_desired[1];
    }

    if((Move_mode & 0b01) == 0) //z channel
    {
        Command_to_pub.Reference_State.position_ref[2] = state_desired[2];
        Command_to_pub.Reference_State.velocity_ref[2] = 0;
    }
    else
    {
        Command_to_pub.Reference_State.position_ref[2] = 0;
        Command_to_pub.Reference_State.velocity_ref[2] = state_desired[2];
    }

    Command_to_pub.Reference_State.acceleration_ref[0] = 0;
    Command_to_pub.Reference_State.acceleration_ref[1] = 0;
    Command_to_pub.Reference_State.acceleration_ref[2] = 0;


    Command_to_pub.Reference_State.yaw_ref = state_desired[3]/180.0*M_PI;
}
