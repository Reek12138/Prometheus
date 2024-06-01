#ifndef XSUGV_BASE_XSUGV_HARDWARE_H
#define XSUGV_BASE_XSUGV_HARDWARE_H

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "xsugv_base/xsugv_interface.h"
#include <string>

namespace xsugv_base
{

  /**
  * Class representing XSUGV hardware, allows for ros_control to modify internal state via joint interfaces
  */
  class XSUGVHardware :
    public hardware_interface::RobotHW
  {
  public:
    XSUGVHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq);

    ~XSUGVHardware();

    void updateJointsFromHardware();

    void writeCommandsToHardware();

  private:

    void registerControlInterfaces();

    double linearToAngular(const double &travel) const;

    double angularToLinear(const double &angle) const;

    void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

    ros::NodeHandle nh_, private_nh_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    // ROS Parameters
    double wheel_base_, wheel_diameter_, max_accel_, max_speed_;

    double polling_timeout_;
    double control_period_;
    ivcu_data_t ugv_data_;

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint
    {
      double position;
      double velocity;
      double effort;
      double velocity_command;

      Joint() :
        position(0), velocity(0), effort(0), velocity_command(0)
      { }
    } joints_[6];
  };

}  // namespace xsugv_base
#endif  // XSUGV_BASE_XSUGV_HARDWARE_H
