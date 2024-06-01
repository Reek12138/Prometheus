#include "xsugv_base/xsugv_hardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

/**
* Control loop for XSUGV, not realtime safe
*/
void controlLoop(xsugv_base::XSUGVHardware &xsugv_hw,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{

  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // Process control loop
  xsugv_hw.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  xsugv_hw.writeCommandsToHardware();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "xsugv_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency, diagnostic_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 50.0);
  private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

  // Initialize robot hardware and link to controller manager
  xsugv_base::XSUGVHardware xsugv_hw(nh, private_nh, control_frequency);
  controller_manager::ControllerManager cm(&xsugv_hw, nh);

  // Setup separate queue and single-threaded spinner to process timer callbacks
  // that interface with XSUGV hardware - libhorizon_legacy not threadsafe. This
  // avoids having to lock around hardware access, but precludes realtime safety
  // in the control loop.
  ros::CallbackQueue xsugv_queue;
  ros::AsyncSpinner xsugv_spinner(1, &xsugv_queue);

  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(
    ros::Duration(1 / control_frequency),
    boost::bind(controlLoop, boost::ref(xsugv_hw), boost::ref(cm), boost::ref(last_time)),
    &xsugv_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  xsugv_spinner.start();

  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();

  return 0;
}
