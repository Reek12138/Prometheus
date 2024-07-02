import os
import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from numpy import inf
from torch.utils.tensorboard import SummaryWriter

from velodyne_env import GazeboEnv
from sac_network import SACContinuous, ReplayBuffer
import math

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
seed = 0

num_step = 500
num_episode = 3000

eval_freq = 30
num_eval_episode = 10
num_eval_step = 100

expl_noise = 1
expl_decay_steps = 500000
expl_min = 0.1
batch_size = 512
discount = 0.99999
tau = 0.005
policy_noise = 0.2
noise_clip = 0.5
policy_freq = 2
buffer_size = 1e6
file_name = "SAC_velodyne"
save_model = True
load_model = True
random_near_obstacle = True
train_frequence = 5

if not os.path.exists("./results"):
    os.makedirs("./results")
if save_model and not os.path.exists("./model"):
    os.makedirs("./model")

environment_dim = 20
env = GazeboEnv(environment_dim)
torch.manual_seed(seed)
np.random.seed(seed)
robot_dim = 4
state_dim = environment_dim + robot_dim
action_dim = 2
max_action = 1

network = SACContinuous(state_dim=state_dim, hidden_dim=512, action_dim=action_dim,
                        actor_lr=1e-4, critic_lr=1e-4, alpha_lr=1e-4,
                        target_entropy=-math.log(2), tau=tau, gamma=discount, device=device)

replay_buffer = ReplayBuffer(buffer_size, seed)
writer = SummaryWriter()
iter_count = 0

def evaluate(eval_ep):
    
    model_path = os.path.join("model")
    network.load_model(model_path, file_name)
    collision_count = 0
    goal_count = 0
    total_reward = 0
    # for eval_episode_i in range(eval_ep):
    eval_episode_i = 0
    while eval_episode_i <= eval_ep:
        
        eval_state = env.reset()
        time.sleep(0.5)
        eval_done = False
        # for eval_step_i in range(num_eval_step):
        eval_step_i = 0
        while eval_step_i <= num_eval_step:
            eval_action = network.get_action(eval_state)
            eval_noise = np.random.normal(0, 0.05, size=eval_action.shape)
            eval_noisy_action = eval_action + eval_noise
            eval_noisy_action = np.clip(eval_noisy_action, -1, 1)
            eval_action_in = [((eval_noisy_action[0] + 1) / 2) , eval_noisy_action[1] * (np.pi / 4)]

            eval_next_state, eval_reward, eval_done, eval_target = env.step(eval_action_in)

            eval_state = eval_next_state

            total_reward += eval_reward
            if eval_done and not eval_target:
                collision_count += 1
            elif eval_target :
                goal_count += 1

            if eval_done :
                break
            else:
                eval_step_i += 1
        
        eval_episode_i += 1

    ave_reward = total_reward / eval_ep
    ave_collision = collision_count / eval_ep
    print(f"{eval_ep}轮验证平均reward {ave_reward:.2f}, 到达目标{goal_count}次，平均碰撞次数 {ave_collision}")
    # print("-------------------------------------------------")
    print("-------------------------TRAINING----------------------------")




# for episode_i in range(num_episode):
episode_i = 0
print("-------------------------TRAINING----------------------------")
while episode_i <= num_episode:
    # 验证
    if episode_i % eval_freq == 0 and episode_i > 2:
        print("------------------------验证中-------------------------")
        evaluate(num_eval_episode)
    # print("-------------------------TRAINING----------------------------")

    state = env.reset()
    time.sleep(0.5)
    done = False
    done_bool = False
    # for step_i in range(max_ep):
    step_i = 0
    while step_i <= num_step:
        # if not done:

        total_step = episode_i * num_step + step_i

        action = network.get_action(state)
        noise = np.random.normal(0, 0.2, size=action.shape)
        noisy_action = action + noise
        noisy_action = np.clip(noisy_action, -1, 1)
        action_in = [((noisy_action[0] + 1) / 2) , noisy_action[1] * (np.pi / 4)]

        next_state, reward, done, target = env.step(action_in)
        done_bool = step_i + 1 == num_step or done
        replay_buffer.add(state, action, reward, done_bool, next_state)
        # if step_i % 2 == 0:
        #     print(f"action_in: [{action_in[0]:.2f}, {action_in[1]:.2f}], reward: {reward:.4f}, state: {state}")

        state = next_state

        if (total_step + 1) % train_frequence == 0 and total_step > batch_size:
            iter_count += 1
            s, a, r, d, ns = replay_buffer.sample_batch(batch_size)
            transition_dict = {'states': s, 'actions': a, 'rewards': r, 'next_states': ns, 'dones': d}
            alpha_loss, av_q = network.update(transition_dict)
            writer.add_scalar("loss", alpha_loss, iter_count)
            writer.add_scalar("av_q", av_q, iter_count)

            model_path = os.path.join("./model")
            network.save_model(model_path, file_name)
        
        if done :
            break
        else:
            step_i += 1

    if done and not target:
        print(f"-------------------------episode {episode_i} 发生碰撞---------------------------")
    elif done and target:
        print(f"-------------------------episode {episode_i} 到达目标---------------------------")
    else:
        print(f"-------------------------episode {episode_i} 未找到目标---------------------------")

    
    episode_i += 1


