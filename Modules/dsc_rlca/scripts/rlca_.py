#!/usr/bin/env python3

import rospy
import paddle
from prometheus_msgs.msg import DroneState, SwarmCommand, DroneStateExp, SwarmCommandExp
import sys, os
import numpy as np
from math import sin, cos, tan, atan2
# sys.path.append(os.path.join(os.path.dirname(__file__), '../DSC-RLCA'))

from sac_agent import Agent
from sac_model import Model


TRAJ_START_FLAG = False

class MultiState:
    def  __init__(self, n) :
        self.n = n
        self.subscribers = []
        self.drone_state = [None]*n

        for i in range(n):
            id = i+1
            sub = rospy.Subscriber(f'/uav{id}/prometheus/drone_state', DroneState, self.state_cb, callback_args=i)
            self.subscribers.append(sub)

    def state_cb(self, data, index):
        self.drone_state[index] = data
    
def generate_path_points(start, end, num_points):
    """
    生成从起点到终点的路径点列表。
    
    参数:
    - start: 起点坐标，格式为 [x_start, y_start]
    - end: 终点坐标，格式为 [x_end, y_end]
    - num_points: 路径点数量，包括起点和终点

    返回:
    - 路径点列表，每个点为 [x, y] 格式
    """
    x_values = np.linspace(start[0], end[0], num_points)
    y_values = np.linspace(start[1], end[1], num_points)
    path_points = [[x, y] for x, y in zip(x_values, y_values)]
    
    return np.array(path_points)

def TrajStart_cb (state_msg):
    global TRAJ_START_FLAG
    if state_msg.source == "traj_start":
        TRAJ_START_FLAG = True

    
def main():
    rospy.init_node('RLCA_node', log_level=rospy.DEBUG)
    rate = rospy.Rate(10)

    # 读取参数
    # rospy.sleep(5) 
    uav_num = 4
    uav_id = 1
    self_id = uav_id -1
    vel = 1.0

    start_point_x = 5
    start_point_y = 5
    end_point_x = -5
    end_point_y = -5
    waypoint_num = 720

    waypoint = generate_path_points(start=[start_point_x, start_point_y], 
                                    end=[end_point_x, end_point_y],
                                    num_points=waypoint_num)

    Multi_State = MultiState(uav_num)
    pub = rospy.Publisher(f'/uav1/prometheus/swarm_command',SwarmCommand, queue_size=10)
    
    # 判断算法开始
    start_listener = rospy.Subscriber(f'/uav{self_id}/prometheus/drone_state', SwarmCommand, TrajStart_cb)

    

    # obs_space = 4 + 5 * (uav_num - 1)
    obs_space = 34
    frames = 3
    act_space = 2

    model = Model(obs_space, frames, act_space)
    save_path = "inference_model_cnn_L"
    param_dict = paddle.load(save_path)
    model.set_state_dict(param_dict)

    uav = Agent(model, 0)

    uav.reset(plan=waypoint)        #[[x1,y1], [x2, y2],........]

    # while TRAJ_START_FLAG == False:
    #     # print("wait for exp start")
    #     rospy.sleep(0.5) 


    print("exp start")
    msg = SwarmCommand()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()
    msg.source = "RLCA_control"
    msg.Mode = SwarmCommand.Move
    msg.Move_mode = SwarmCommand.XY_VEL_Z_POS
    msg.position_ref = [0, 0, 3]
    msg.velocity_ref = [-0.707, -0.707, 0.0]
    msg.acceleration_ref = [0.0, 0.0, 0.0]
    msg.yaw_ref = 0.0
    msg.yaw_rate_ref = 0.0

    pub.publish(msg)


    # while not rospy.is_shutdown() and TRAJ_START_FLAG == True:
    while not rospy.is_shutdown() :
        obses = []
        for i in range(uav_num):
            if Multi_State.drone_state[i] is None:
                rospy.logwarn(f"Waiting for UAV {i} state data...")
                rospy.sleep(0.5)  # 添加稍微的延迟避免反复打印
                continue
            # 获取状态数据
            obs = [
                Multi_State.drone_state[i].position[0],
                Multi_State.drone_state[i].position[1],
                atan2(Multi_State.drone_state[i].velocity[1], Multi_State.drone_state[i].velocity[0]),
                np.linalg.norm([Multi_State.drone_state[i].velocity[0], Multi_State.drone_state[i].velocity[1]])
            ]
            obses.append(obs)

        
        # for j in range (9-uav_num):
        #     obs = [100, 100, 0, 0]
        #     obses.append(obs)

        obses_np = np.array(obses)
        if obses_np.size == 0:
            rospy.logerr("obses_np is empty. Skipping this iteration.")
            rate.sleep()
            continue

        action,_,_,_,_ = uav.action(obses_np[self_id],np.delete(obses_np,self_id,axis=0),action_type = "predict")
        print("action :" , action)

        theta = atan2(Multi_State.drone_state[0].velocity[1], Multi_State.drone_state[0].velocity[0])
        theta += action[0] * 2.5
        theta = theta % (2* np.pi)
        vx = vel * cos(theta)
        vy = vel * sin(theta)

        msg = SwarmCommand()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.source = "RLCA_control"
        msg.Mode = SwarmCommand.Move
        msg.Move_mode = SwarmCommand.XY_VEL_Z_POS
        msg.position_ref = [0, 0, 3]
        msg.velocity_ref = [vx, vy, 0.0]
        msg.acceleration_ref = [0.0, 0.0, 0.0]
        msg.yaw_ref = 0.0
        msg.yaw_rate_ref = 0.0

        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr(f"Exception occurred: {e}")

