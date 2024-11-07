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

move_type = 3

def euler_to_quaternion(roll, pitch, yaw):
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

def pose_publisher_line():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'car_landing_pad'
    # pose_msg.model_name = 'xsugv_c6_ros_/'
    rate = rospy.Rate(100)
    linear_vel = 1.7
    time = 0.0
    while not rospy.is_shutdown():
        pos = -12 + time * linear_vel
        time = time + 0.01
        if pos > 30.0:
            time = 0
        else:
            pass
        
        pose_msg.pose.position.x = pos
        pose_msg.pose.position.y = -2.0
        pose_msg.pose.position.z = 0.01
        # quaternion = euler_to_quaternion(0, 0, pi/6)
        quaternion = euler_to_quaternion(0, 0, 0)
        pose_msg.pose.orientation = Quaternion(*quaternion)
        
        pub.publish(pose_msg)
        print('Pos_x :',pose_msg.pose.position.x)
        print('Pos_y :',pose_msg.pose.position.y)
        rate.sleep()


def pose_publisher_circle():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'car_landing_pad'
    rate = rospy.Rate(100)
    linear_vel = 0.8
    circle_radius = 3.0
    omega = math.fabs(linear_vel / circle_radius)
    time = 0.0
    while not rospy.is_shutdown():
        angle = time * omega
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        time = time + 0.01
        pose_msg.pose.position.x = circle_radius*cos_angle
        pose_msg.pose.position.y = circle_radius*sin_angle
        pose_msg.pose.position.z = 0.01
        # quaternion = euler_to_quaternion(0, 0, math.atan2(circle_radius*sin_angle, circle_radius*cos_angle))  # 使物体面向运动方向
        quaternion = euler_to_quaternion(0, 0, 0)  # 使物体面向运动方向
        pose_msg.pose.orientation = quaternion

        pub.publish(pose_msg)
        print('Pos_x :',pose_msg.pose.position.x)
        print('Pos_y :',pose_msg.pose.position.y)
        rate.sleep()

def pose_publisher_acceleration():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'car_landing_pad'
    rate = rospy.Rate(100)
    max_speed = 1.6 # 最大速度
    acceleration = 0.2  # 加速度
    pos = -11.0
    time = 0.0
    speed = 0.0
    accelerating = True  # 标记是否在加速

    while not rospy.is_shutdown():
        if accelerating:
            speed += acceleration * 0.01
            if speed >= max_speed:
                speed = max_speed
                accelerating = False
        else:
            speed -= acceleration * 0.01
            if speed <= 0:
                speed = 0
                accelerating = True

        pos += speed * 0.01
        time += 0.01
        if pos > 30.0:
            pos = -11.0
            time = 0.0
            speed = 0.0
            accelerating = True
        
        pose_msg.pose.position.x = pos
        pose_msg.pose.position.y = -2.0
        pose_msg.pose.position.z = 0.01
        quaternion = euler_to_quaternion(0, 0, 0)
        pose_msg.pose.orientation = Quaternion(*quaternion)
        
        pub.publish(pose_msg)
        print('Pos_x :', pose_msg.pose.position.x)
        print('Pos_y :', pose_msg.pose.position.y)
        print('Speed :', speed)
        rate.sleep()


def pose_publisher_s_shape_trajectory():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'car_landing_pad'
    rate = rospy.Rate(100)
    
    # S 形轨迹参数
    amplitude = 2.0  # S 形曲线的振幅（相当于曲线的高度）
    wavelength = 10.0  # 波长（S 形的一个周期长度）
    linear_speed = 0.7 # 线速度
    z_height = 0.01  # 固定的 z 轴高度

    
    time = 0.0
    
    while not rospy.is_shutdown():
        # 计算当前的 S 形轨迹位置
        initial_x = -9.0  # 设定起始的 x 坐标
        initial_y = -2.0  # 设定起始的 y 坐标

        # 计算当前的 S 形轨迹位置
        x = initial_x + linear_speed * time  # 在起始点的基础上前进
        y = initial_y + amplitude * math.sin((2 * math.pi / wavelength) * x)

        
        # 更新位置信息
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z_height  # z 轴保持不变
        
        # 姿态更新为水平
        # quaternion = euler_to_quaternion(0, 0, math.atan2(y, x))  # 使物体面向运动方向
        quaternion = euler_to_quaternion(0, 0, 0)  # 使物体面向运动方向
        pose_msg.pose.orientation = Quaternion(*quaternion)
        
        # 发布消息
        pub.publish(pose_msg)
        
        # 打印位置信息和时间
        print('Pos_x :', pose_msg.pose.position.x)
        print('Pos_y :', pose_msg.pose.position.y)
        
        # 时间推进
        time += 0.01
        rate.sleep()

if __name__ == '__main__':
      rospy.init_node('car_pose_publisher')
      try:
          if move_type == 0:
            pose_publisher_circle()
          elif move_type == 1:
            pose_publisher_line()
          elif move_type == 2:    
              pose_publisher_acceleration()
          elif move_type == 3:
              pose_publisher_s_shape_trajectory() 
          
      except rospy.ROSInterruptException:
          pass
