#!/bin/bash

# 启动第一个launch文件，并等待10秒
gnome-terminal -- bash -c "roslaunch prometheus_gazebo sitl_nmpc_uav.launch; sleep 8"

# 等待一段时间确保第一个launch文件完全启动
sleep 10

# 启动第二个launch文件，并等待5秒
gnome-terminal -- bash -c "roslaunch prometheus_control takeoff.launch; sleep 5"

# 等待一段时间以确保第二个launch文件完全启动
sleep 10

# 启动第三个launch文件
gnome-terminal -- bash -c "roslaunch nmpc_ctr uav_nmpc_2.launch"
