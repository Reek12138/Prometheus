#!/bin/bash

source ~/.bashrc

/usr/bin/gnome-terminal --name=C6_ROS -- /bin/bash -i -c "source ~/.bashrc && /opt/ros/melodic/bin/roslaunch xsugv robot.launch"

sleep 10

/usr/bin/gnome-terminal --name=C6_ROS_VIEW -- /bin/bash -i -c "source ~/.bashrc && /opt/ros/melodic/bin/roslaunch xsugv_viz view_robot.launch"

