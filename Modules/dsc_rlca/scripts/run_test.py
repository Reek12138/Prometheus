import paddle
import numpy as np
import math
import time

from Path_planning import Plan
from plot2DEnv import plot2DEnv
from sac_model import Model
from sac_agent import Agent   #导入智能体
# from sac import SAC           #导入算法
N = 4

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

def run_evaluate_episodes(model, env, eval_episodes):


    for eps in range(eval_episodes):
        #生成路径plan
        mission_plan = Plan(N)
        env_message,plan = mission_plan.env_unchanged(1)

        #初始化
        # obs = env.reset(env_message[0],env_message[1],env_message[2],env_message[3],env_message[4])
        agents = [[] for x in range(env.N)]
        start_point = [[50, 50], [50, -50], [-50, -50], [-50, 50]]
        end_point = [[-50, -50], [-50, 50], [50, 50], [50, -50]]
        plan = []
        for i in range(env.N):
            agents[i] = Agent(model,i)      
            plan_ = generate_path_points(start_point[i], end_point[i], 720)
            plan.append(plan_)
            # print(plan)
            # agents[i].reset(plan[i])
            agents[i].reset(plan_)
        
        plan = np.array(plan)
        env_start = plan[:,0]
        env_goal = plan[:,-1]
        env_v = [1.0 for x in range(env.N)]#velocity
        env_angle = np.arctan2(plan[:,-1,1]-plan[:,0,1],plan[:,-1,0]-plan[:,0,0])
        env_boundary = [-30,30,-30,30]
        obs = env.reset(env_start,env_goal,env_v,env_angle,env_boundary)
            
        action = np.zeros((env.N,act_space))

        done = np.full((N, ), False, dtype=bool)
        episode_steps = 0
        while not done.all():
            env.render()
            episode_steps += 1
            for i in range(env.N):
                # start = time.time()
                action[i],_,_,_,_ = agents[i].action(obs[i],np.delete(obs,i,axis=0),action_type = "predict")
                
                # end = time.time()
                # print("cal_time",end - start)
            # action[1] = np.zeros(2)       #不合作飞机
            # action[3] = np.zeros(2)       #不合作飞机
            # action[5] = np.zeros(2)       #不合作飞机
            # action[7] = np.zeros(2)       #不合作飞机
            # print(action)
            obs, _, done,  _ = env.step(action)

            if episode_steps >= 200:
                break
        print("episode_step", episode_steps)
        env.close()
        
        # data_save_path ='D:\HANJIALE\code\ORCA1\\agent_orca\paper_data\\traj\data_rl_fixed_random.csv'
        # # data_save_path = f'D:\\HANJIALE\\code\\ORCA1\\agent_orca\\data_statistics\\RL_circle_8\\data_rl_{eps}.csv'
        # env.data_save(data_save_path)     #保存数据
        

    return

if __name__ == "__main__":

    env = plot2DEnv(N=N)

    obs_N = 6                          # 观察到其他无人机数目
    frames = 3
    obs_space = 4+5*obs_N     # 状态空间维数
    act_space = 2
    model = Model(obs_space, frames ,act_space)
    
    save_path = "inference_model_cnn_L"
    param_dict = paddle.load(save_path)
    model.set_state_dict(param_dict)

    run_evaluate_episodes(model, env, 1)
    

