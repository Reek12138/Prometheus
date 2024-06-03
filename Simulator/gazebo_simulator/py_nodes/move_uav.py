#! /usr/bin/env python
# -*- coding: utf-8 -*-
#author : Colin Lee
#email  : lcyfly1@163.com
#description  :  control qrcode movement 
import rospy
import math
from gazebo_msgs.msg import ModelState

from geometry_msgs.msg import Quaternion
import tf.transformations
from math import pi


def euler_to_quaternion(roll, pitch, yaw):
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

def pose_publisher():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    
    pose_msg.model_name = 'p450_3Dlidar'
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        delta = 0.01
        linear_vel = 1.0
        angular_vel = 0.5
        
        pose_msg.pose.position.x += linear_vel * delta
        pose_msg.pose.position.y = -2.0
        pose_msg.pose.position.z = 3.0
        # quaternion = euler_to_quaternion(0, 0, pi/6)
        quaternion = euler_to_quaternion(0, 0, 0)
        pose_msg.pose.orientation = Quaternion(*quaternion)

        # pose_msg.twist.linear.x = 1.0
        # pose_msg.twist.angular.z = 1.0



        
        pub.publish(pose_msg)
        # print('Pos_x :',pose_msg.pose.position.x)
        # print('Pos_y :',pose_msg.pose.position.y)
        rate.sleep()



if __name__ == '__main__':
      rospy.init_node('car_pose_publisher')
      try:
        pose_publisher()
          
      except rospy.ROSInterruptException:
        # print("???????")
          pass
