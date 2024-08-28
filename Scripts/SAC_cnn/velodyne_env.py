import math
import os
import random
import subprocess
import time
from os import path

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from collections import deque
import torch
from torch import nn

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
# TIME_DELTA = 0.01
TIME_DELTA = 0.15


# Check if the random goal position is located on an obstacle and do not accept it if it is
def check_pos(x, y):
    goal_ok = True

    # 新增障碍物的检查
    if 2.5 < x < 7 and 2.5 < y < 7:
        goal_ok = False

    if -7.5 < x < -2.5 and 2.5 < y < 7:
        goal_ok = False

    if -4 < x < 4 and -6 < y < -2:
        goal_ok = False

    # 横条的检查
    line_barrier_0 = {'x_range': (-6.45989, -3.45989), 'y_range': (-2.24882, -1.24882)}
    line_barrier_1 = {'x_range': (-3.3279, -0.3279), 'y_range': (-1.38953, 0.61047)}
    line_barrier_2 = {'x_range': (-7.33814, -4.33814), 'y_range': (-8.1984, -5.1984)}

    barriers = [line_barrier_0, line_barrier_1, line_barrier_2]

    for barrier in barriers:
        if barrier['x_range'][0] < x < barrier['x_range'][1] and barrier['y_range'][0] < y < barrier['y_range'][1]:
            goal_ok = False
            break

    return goal_ok


class LaserCNN(nn.Module):
    def __init__(self, output_features):
        super(LaserCNN, self).__init__()
        self.conv1 = nn.Conv1d(3, 16, kernel_size=5, stride=1, padding=2)  # 使用3个输入通道
        self.conv2 = nn.Conv1d(16, 32, kernel_size=5, stride=1, padding=2)
        self.pool = nn.AdaptiveAvgPool1d(1)  # 使用自适应平均池化到1
        self.fc = nn.Linear(32, output_features)

    def forward(self, x):
        x = torch.relu(self.conv1(x))
        x = torch.relu(self.conv2(x))
        x = self.pool(x)
        x = x.view(x.size(0), -1)  # 展平
        x = self.fc(x)
        return x


class GazeboEnv:
    """Superclass for all Gazebo environments."""

    def __init__(self, environment_dim):
        self.environment_dim = environment_dim
        self.odom_x = 0
        self.odom_y = 0

        self.goal_x = 1
        self.goal_y = 0.0

        self.upper = 5.0
        self.lower = -5.0
        self.velodyne_data = np.ones(self.environment_dim) * 10
        self.last_odom = None

        self.set_self_state = ModelState()
        self.set_self_state.model_name = "xsugv_c6_ros_/"
        # self.set_self_state.model_name = "p3dx"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.0
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0
        self.last_target_dis = -math.inf

        self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / self.environment_dim]]
        for m in range(self.environment_dim - 1):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        self.gaps[-1][-1] += 0.03

       
        rospy.init_node("gym", anonymous=True)
        
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        self.set_state = rospy.Publisher(
            "gazebo/set_model_state", ModelState, queue_size=10
        )
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.publisher = rospy.Publisher("goal_point", MarkerArray, queue_size=3)
        self.publisher2 = rospy.Publisher("linear_velocity", MarkerArray, queue_size=1)
        self.publisher3 = rospy.Publisher("angular_velocity", MarkerArray, queue_size=1)
        self.velodyne = rospy.Subscriber(
            "/velodyne_points", PointCloud2, self.velodyne_callback, queue_size=1
        )
        self.odom = rospy.Subscriber(
            "/xsugv_velocity_controller/odom", Odometry, self.odom_callback, queue_size=1
            
        )

        self.laser_deque = deque(maxlen=3)

    # Read velodyne pointcloud and turn it into distance data, then select the minimum value for each angle
    # range as state representation
    def velodyne_callback(self, v):
        data = list(pc2.read_points(v, skip_nans=False, field_names=("x", "y", "z")))
        self.velodyne_data = np.ones(self.environment_dim) * 10
        for i in range(len(data)):
            if data[i][2] > -0.35:
                dot = data[i][0] * 1 + data[i][1] * 0
                mag1 = math.sqrt(math.pow(data[i][0], 2) + math.pow(data[i][1], 2))
                mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
                beta = math.acos(dot / (mag1 * mag2)) * np.sign(data[i][1])
                dist = math.sqrt(data[i][0] ** 2 + data[i][1] ** 2 + data[i][2] ** 2)

                for j in range(len(self.gaps)):
                    if self.gaps[j][0] <= beta < self.gaps[j][1]:
                        self.velodyne_data[j] = min(self.velodyne_data[j], dist)
                        break
        # print("Velodyne data: ", self.velodyne_data)

    def odom_callback(self, od_data):
        self.last_odom = od_data

    # Perform an action and read a new state
    def step(self, action):
        target = False

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.vel_pub.publish(vel_cmd)
        self.publish_markers(action)

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pass
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        # read velodyne laser state
        done, collision, min_laser = self.observe_collision(self.velodyne_data)
        # print("min_laser : ",min_laser  )
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        self.laser_deque.append(laser_state)
        laser_array = np.stack(self.laser_deque, axis=0)


        # Calculate robot heading from odometry data
        self.odom_x = self.last_odom.pose.pose.position.x
        self.odom_y = self.last_odom.pose.pose.position.y
        quaternion = Quaternion(
            self.last_odom.pose.pose.orientation.w,
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        if distance < GOAL_REACHED_DIST:
            target = True
            done = True

        robot_state = [distance, theta, action[0], action[1]]
        # state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, collision, action, min_laser, distance, self.last_target_dis)

        self.last_target_dis = distance

        return laser_array, robot_state, reward, done, target

    def reset(self, start_x=None, start_y=None, start_angle=None):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.reset_proxy()

        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")

        # angle = np.random.uniform(-np.pi, np.pi)
        # quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        # object_state = self.set_self_state

        # 自定义修改
        angle = start_angle if start_angle is not None else np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        object_state = self.set_self_state

        # Set position
        if start_x is not None and start_y is not None:
            x = start_x
            y = start_y
        else:
            position_ok = False
            while not position_ok:
                x = np.random.uniform(-4.5, 4.5)
                y = np.random.uniform(-4.5, 4.5)
                position_ok = check_pos(x, y)
        # x = 0
        # y = 0
        # position_ok = False
        # while not position_ok:
        #     x = np.random.uniform(-4.5, 4.5)
        #     y = np.random.uniform(-4.5, 4.5)
        #     position_ok = check_pos(x, y)

        object_state.pose.position.x = x
        object_state.pose.position.y = y
        object_state.pose.position.z = 0
        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)

        self.odom_x = object_state.pose.position.x
        self.odom_y = object_state.pose.position.y

        # set a random goal in empty space in environment
        self.change_goal()
        # randomly scatter boxes in the environment
        self.random_box()
        self.set_barrier_positions()
        self.publish_markers([0.0, 0.0])
        


        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        self.laser_deque.clear()
        self.laser_deque.append(laser_state)
        self.laser_deque.append(laser_state)
        self.laser_deque.append(laser_state)
        laser_array = np.stack(self.laser_deque, axis=0)

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )
        self.last_target_dis = distance

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        robot_state = [distance, theta, 0.0, 0.0]
        # state = np.append(laser_state, robot_state)
        return laser_array, robot_state

    def change_goal(self, goal_x=None, goal_y=None):
        if goal_x is not None and goal_y is not None:
            self.goal_x = goal_x
            self.goal_y = goal_y
        else:
            goal_ok = False

            while not goal_ok:
                self.goal_x = random.uniform(-10, 10)
                self.goal_y = random.uniform(-10, 10)
                goal_ok = check_pos(self.goal_x, self.goal_y)

       

   
    def random_box(self):
        # Randomly change the location of the boxes in the environment on each reset to randomize the training environment
        existing_positions = []
        for i in range(4):
            name = "cube_20k_" + str(i)

            x = 0
            y = 0
            box_ok = False
            while not box_ok:
                x = np.random.uniform(-9, 9)
                y = np.random.uniform(-9, 9)

                box_ok = check_pos(x, y)
                distance_to_robot = np.linalg.norm([x - self.odom_x, y - self.odom_y])
                distance_to_goal = np.linalg.norm([x - self.goal_x, y - self.goal_y])

                # Check if the new position is too close to existing positions
                min_distance = 1.0  # Minimum distance between boxes
                for pos in existing_positions:
                    if np.linalg.norm([x - pos[0], y - pos[1]]) < min_distance:
                        box_ok = False
                        break

                if distance_to_robot < 1.5 or distance_to_goal < 1.5:
                    box_ok = False

            existing_positions.append((x, y))

            box_state = ModelState()
            box_state.model_name = name
            box_state.pose.position.x = x
            box_state.pose.position.y = y
            box_state.pose.position.z = 0.0
            box_state.pose.orientation.x = 0.0
            box_state.pose.orientation.y = 0.0
            box_state.pose.orientation.z = 0.0
            box_state.pose.orientation.w = 1.0
            self.set_state.publish(box_state)

    def set_barrier_positions(self):
        barriers = [
            {"name": "line_barrier_2", "x": -6.13183, "y": -2.24401, "z": 0, "orientation": 0.512506},
            {"name": "line_barrier_0", "x": 5.61972, "y": -3.07262, "z": 0, "orientation": 1.95606},
            {"name": "line_barrier_1", "x": 6.79538, "y": -6.97677, "z": 0, "orientation": 0},
        ]

        for barrier in barriers:
            state = ModelState()
            state.model_name = barrier["name"]
            state.pose.position.x = barrier["x"]
            state.pose.position.y = barrier["y"]
            state.pose.position.z = barrier["z"]
            quaternion = Quaternion.from_euler(0.0, 0.0, barrier["orientation"])
            state.pose.orientation.x = quaternion.x
            state.pose.orientation.y = quaternion.y
            state.pose.orientation.z = quaternion.z
            state.pose.orientation.w = quaternion.w
            self.set_state.publish(state)

    def publish_markers(self, action):
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0

        markerArray.markers.append(marker)

        self.publisher.publish(markerArray)

        markerArray2 = MarkerArray()
        marker2 = Marker()
        marker2.header.frame_id = "odom"
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.scale.x = abs(action[0])
        marker2.scale.y = 0.1
        marker2.scale.z = 0.01
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 5
        marker2.pose.position.y = 0
        marker2.pose.position.z = 0

        markerArray2.markers.append(marker2)
        self.publisher2.publish(markerArray2)

        markerArray3 = MarkerArray()
        marker3 = Marker()
        marker3.header.frame_id = "odom"
        marker3.type = marker.CUBE
        marker3.action = marker.ADD
        marker3.scale.x = abs(action[1])
        marker3.scale.y = 0.1
        marker3.scale.z = 0.01
        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.pose.position.x = 5
        marker3.pose.position.y = 0.2
        marker3.pose.position.z = 0

        markerArray3.markers.append(marker3)
        self.publisher3.publish(markerArray3)

    @staticmethod
    def observe_collision(laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        # print(min_laser)
        if min_laser < COLLISION_DIST:
            return True, True, min_laser
        return False, False, min_laser

    @staticmethod
    def get_reward(target, collision, action, min_laser, target_distance, last_target_distance):
        if target:
            return 100.0
        elif collision:
            return -100.0
        else:
            r3 = lambda x: 1.0 - x if x < 1.0 else 0.0
            # r3 = lambda x: -((1.0/(x + 1))-0.5)*2 if x < 1.0 else 0.0
            r4 = -(target_distance - last_target_distance)
            return (action[0])*2 - abs(action[1])*2  - r3(min_laser)  + r4
