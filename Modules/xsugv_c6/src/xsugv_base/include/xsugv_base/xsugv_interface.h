#ifndef XSUGV_BASE_XSUGV_INTERFACE_H
#define XSUGV_BASE_XSUGV_INTERFACE_H

#include <stdint.h>
#include <ros/ros.h>

namespace xsugv_base
{

typedef struct {
    double lr_wheel_speed;
    double rr_wheel_speed;
    double linear_velocity;
    double front_wheel_angle;

    double voltage;
    double current;
    double battery;
    double temperature;

    unsigned char driving_mode;
    unsigned char gear;

    bool turn_left_light;
    bool turn_right_light;

    uint64_t ivcu_state1_timestamp;
    uint64_t ivcu_state2_timestamp;
    uint64_t ivcu_imu_timestamp;
    uint64_t ivcu_gps_timestamp;
} ivcu_data_t;

void ugvSetCmd(double linear_velocity, double angular_velocity);
void ugvGetData(ivcu_data_t *pd);
bool ugvStart(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
void ugvStop();

} // namespace xsugv_base

#endif // XSUGV_BASE_XSUGV_INTERFACE_H

