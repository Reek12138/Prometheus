#include "xsugv_base/xsugv_hardware.h"
#include <boost/assign/list_of.hpp>
#include <stdlib.h>

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
};

namespace xsugv_base
{

  /**
  * Initialize XSUGV hardware
  */
  XSUGVHardware::XSUGVHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
    :
    nh_(nh),
    private_nh_(private_nh),
    control_period_(1 / target_control_freq)
  {
    private_nh_.param<double>("wheel_base", wheel_base_, 0.54);
    private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.2);
    private_nh_.param<double>("max_accel", max_accel_, 5.0);
    private_nh_.param<double>("max_speed", max_speed_, 1.388889);
    private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

    ugvStart(nh, private_nh);
    registerControlInterfaces();
  }

  XSUGVHardware::~XSUGVHardware()
  {
    ugvStop();
  }

  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void XSUGVHardware::registerControlInterfaces()
  {
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")("front_right_wheel")
                                                      ("middle_left_wheel")("middle_right_wheel")
                                                      ("rear_left_wheel")("rear_right_wheel");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity,
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
  }

  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void XSUGVHardware::updateJointsFromHardware()
  {
    	ugvGetData(&ugv_data_);
        ROS_DEBUG_STREAM("Received linear speed information (L:" << ugv_data_.lr_wheel_speed << " R:" << ugv_data_.rr_wheel_speed << ")" << ", DT=" << control_period_);
	for (int i = 0; i < 6; i++) {
		joints_[i].velocity = linearToAngular((i % 2) == LEFT ? ugv_data_.lr_wheel_speed : ugv_data_.rr_wheel_speed);
		double delta = joints_[i].velocity * control_period_;
		if (std::abs(delta) < 1.0) {
		  joints_[i].position += delta;
		} else {
		  ROS_DEBUG("Dropping overflow measurement from encoder");
		}
	}
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void XSUGVHardware::writeCommandsToHardware()
  {
    double diff_speed_left = angularToLinear(joints_[LEFT].velocity_command);
    double diff_speed_right = angularToLinear(joints_[RIGHT].velocity_command);

    limitDifferentialSpeed(diff_speed_left, diff_speed_right);
    ROS_DEBUG_STREAM("vl="<<diff_speed_left << ", vr=" <<diff_speed_right);
    // 底盘协议约定：线速度域传递左轮速，角度域传递右轮速
    ugvSetCmd(diff_speed_left, diff_speed_right);
  }

  /**
  * Scale left and right speed outputs to maintain ros_control's desired trajectory without saturating the outputs
  */
  void XSUGVHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > max_speed_)
    {
      diff_speed_left *= max_speed_ / large_speed;
      diff_speed_right *= max_speed_ / large_speed;
    }
  }

  /**
  * XSUGV reports travel in metres, need radians for ros_control RobotHW
  */
  double XSUGVHardware::linearToAngular(const double &travel) const
  {
    return travel / wheel_diameter_ * 2;
  }

  /**
  * RobotHW provides velocity command in rad/s, XSUGV needs m/s,
  */
  double XSUGVHardware::angularToLinear(const double &angle) const
  {
    return angle * wheel_diameter_ / 2;
  }


}  // namespace xsugv_base
