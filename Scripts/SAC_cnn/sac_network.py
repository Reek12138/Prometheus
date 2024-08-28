import torch
from torch import nn
from torch.nn import functional as F
import numpy as np
import collections
import random
import math
import os
from torch.distributions import Normal
from collections import deque




class ReplayBuffer(object):
    def __init__(self, buffer_size, random_seed=123):
        """
        The right side of the deque contains the most recent experiences
        """
        self.buffer_size = buffer_size
        self.count = 0
        self.buffer = deque()
        random.seed(random_seed)

    def add(self, s, a, r, t, s2, l, l2):
        experience = (s, a, r, t, s2, l, l2)
        if self.count < self.buffer_size:
            self.buffer.append(experience)
            self.count += 1
        else:
            self.buffer.popleft()
            self.buffer.append(experience)

    def size(self):
        return self.count

    def sample_batch(self, batch_size):
        batch = []

        if self.count < batch_size:
            batch = random.sample(self.buffer, self.count)
        else:
            batch = random.sample(self.buffer, batch_size)

        s_batch = np.array([_[0] for _ in batch])
        a_batch = np.array([_[1] for _ in batch])
        r_batch = np.array([_[2] for _ in batch]).reshape(-1, 1)
        t_batch = np.array([_[3] for _ in batch]).reshape(-1, 1)
        s2_batch = np.array([_[4] for _ in batch])
        l_batch = np.array([_[5] for _ in batch])
        l2_batch = np.array([_[6] for _ in batch])

        return s_batch, a_batch, r_batch, t_batch, s2_batch, l_batch, l2_batch

    def clear(self):
        self.buffer.clear()
        self.count = 0


class LaserCNN(nn.Module):
    def __init__(self, input_channels, output_features):
        super(LaserCNN, self).__init__()
        self.conv1 = nn.Conv1d(input_channels, 16, kernel_size=5, stride=1, padding=2)
        self.conv2 = nn.Conv1d(16, 32, kernel_size=5, stride=1, padding=2)
        self.pool = nn.AdaptiveAvgPool1d(1)
        self.fc = nn.Linear(32, output_features)

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = self.pool(x).view(x.size(0), -1)
        x = self.fc(x)
        return x


class PolicyNetContinuous(nn.Module):
    def __init__(self,state_dim,hidden_dim,action_dim, cnn_output_dim):
        super(PolicyNetContinuous,self).__init__()
        self.cnn = LaserCNN(3, cnn_output_dim)
        self.fc1=nn.Linear(state_dim,hidden_dim)
        self.fc_mu=nn.Linear(hidden_dim,action_dim)
        self.fc_std=nn.Linear(hidden_dim,action_dim)
        
        
    def forward(self, state, laser_data):
        laser_features = self.cnn(laser_data)
        combined_input = torch.cat([state, laser_features], dim=1)
        x = F.relu(self.fc1(combined_input))
        # x=F.relu(self.fc1(x))
        mu=self.fc_mu(x)
        std=F.softplus(self.fc_std(x))
        #创建一个正态分布对象，作为动作空间
        dist=Normal(mu,std)
        #动作空间采样，得到样本，样本是动作值，代表连续空间中对应的动作
        normal_sample=dist.rsample()  #rsample()是重参数化采样
        #计算样本的对数概率密度
        log_prob=dist.log_prob(normal_sample)
        #将动作约数在[-1,1]
        action=torch.tanh(normal_sample)
        #计算tanh_normal分布的对数概率密度
#限制动作范围会影响到动作的概率密度函数。这是因为 tanh 函数的导数在边界点上接近于零，这可能导致在这些点上计算的概率密度非常小，甚至接近于零。这会导致梯度消失，从而影响模型的训练效果。
#为了解决这个问题，可以使用公式 log(1 - tanh^2(x) + ε) 来重新计算对数概率密度，其中 ε 是一个较小的常数（在这里是 1e-7），用于避免取对数时的除零错误。这样可以保持对数概率密度的合理值，并避免梯度消失的问题。因此，在该代码中，使用该公式重新计算 log_prob。
        log_prob=log_prob - torch.log(1-torch.tanh(action).pow(2) + 1e-7)
        log_prob = log_prob.sum(dim=-1, keepdim=True)

        return action, log_prob

    def save_checkpoint(self, checkpoint_file):
        torch.save(self.state_dict(), checkpoint_file)

    def load_checkpoint(self, checkpoint_file):
        self.load_state_dict(torch.load(checkpoint_file))


class QValueNetContinuous(nn.Module):
    def __init__(self,state_dim,hidden_dim,action_dim, cnn_output_dim):
        super(QValueNetContinuous,self).__init__()
        self.cnn = LaserCNN(3, cnn_output_dim)
        self.fc1=nn.Linear(state_dim+action_dim + cnn_output_dim,hidden_dim)
        self.fc2=nn.Linear(hidden_dim,hidden_dim)
        self.fc_out=nn.Linear(hidden_dim,1)
        
    def forward(self, state, action, laser_data):
        laser_features = self.cnn(laser_data)
        combined_input = torch.cat([state, action, laser_features], dim=1)
        x = F.relu(self.fc1(combined_input))
        x = F.relu(self.fc2(x))
        output = self.fc_out(x)
        return output
    
    def save_checkpoint(self, checkpoint_file):
        torch.save(self.state_dict(), checkpoint_file)

    def load_checkpoint(self, checkpoint_file):
        self.load_state_dict(torch.load(checkpoint_file))

class SACContinuous:
    """处理连续动作的SAC算法"""
    def __init__(self,state_dim,hidden_dim,action_dim,actor_lr,critic_lr,alpha_lr,target_entropy,tau,gamma,device,cnn_output_dim):
        self.laser_cnn = LaserCNN(3, 20).to(device)
        self.actor=PolicyNetContinuous(state_dim,hidden_dim,action_dim,cnn_output_dim).to(device)  #策略网络
        self.critic_1=QValueNetContinuous(state_dim,hidden_dim,action_dim,cnn_output_dim).to(device)  #第一个Q网络
        self.critic_2=QValueNetContinuous(state_dim,hidden_dim,action_dim,cnn_output_dim).to(device)  #第二个Q网络
        self.target_critic_1=QValueNetContinuous(state_dim,hidden_dim,action_dim,cnn_output_dim).to(device)  #第一个目标Q网络
        self.target_critic_2=QValueNetContinuous(state_dim,hidden_dim,action_dim,cnn_output_dim).to(device)  #第二个目标Q网络
        #令目标Q网络的参数和Q网络一样
        self.target_critic_1.load_state_dict(self.critic_1.state_dict())
        self.target_critic_2.load_state_dict(self.critic_2.state_dict())
        # self.actor_optimizer=torch.optim.Adam(self.actor.parameters(),lr=actor_lr)
        # self.critic_1_optimizer=torch.optim.Adam(self.critic_1.parameters(),lr=critic_lr)
        # self.critic_2_optimizer=torch.optim.Adam(self.critic_2.parameters(),lr=critic_lr)

         # 将CNN的参数也包括在优化器中
        self.actor_optimizer = torch.optim.Adam(
            [
                {'params': self.actor.parameters()},
                {'params': self.actor.cnn.parameters()}  # 确保CNN参数也被更新
            ], 
            lr=actor_lr
        )

        self.critic_1_optimizer = torch.optim.Adam(
            [
                {'params': self.critic_1.parameters()},
                {'params': self.critic_1.cnn.parameters()}  # 确保CNN参数也被更新
            ],
            lr=critic_lr
        )

        self.critic_2_optimizer = torch.optim.Adam(
            [
                {'params': self.critic_2.parameters()},
                {'params': self.critic_2.cnn.parameters()}  # 确保CNN参数也被更新
            ],
            lr=critic_lr
        )

        #使用alpha的log值，可以使训练结果比较稳定
        self.log_alpha=torch.tensor(np.log(0.01),dtype=torch.float)
        self.log_alpha.requires_grad=True  #可以对alpha求梯度
        self.log_alpha_optimizer=torch.optim.Adam([self.log_alpha],lr=alpha_lr)
        self.target_entropy=target_entropy  #目标熵的大小
        self.gamma=gamma
        self.tau=tau
        self.device=device
        
    def get_action(self,state, laser_data):
        state=torch.tensor(np.array(state),dtype=torch.float).to(self.device)
        laser_data = torch.tensor(laser_data, dtype= torch.float).to(self.device)
        action=self.actor(state, laser_data)[0]
        action = action.cpu().detach().numpy().flatten()
        return action
    
    def calc_target(self,rewards,next_states,dones, next_laser_data):  #计算目标Q值
        next_actions,log_prob=self.actor(next_states, next_laser_data)
        #计算熵，注意这里是有个负号的
        entropy=-log_prob
        q1_value=self.target_critic_1(next_states,next_actions)
        q2_value=self.target_critic_2(next_states,next_actions)
        #注意entropy自带负号
        next_value=torch.min(q1_value,q2_value) + self.log_alpha.exp() * entropy
         # 打印调试信息
        # print(f"q1_value shape: {q1_value.shape}, q2_value shape: {q2_value.shape}")
    
        # print(f"min_next_q_values shape: {next_value.shape}")
    
        td_target=rewards + self.gamma * next_value *(1-dones)
        # 打印调试信息
        # print(f"td_target shape: {td_target.shape}")
        return td_target
    
    def soft_update(self,net,target_net):
        for param_target, param in zip(target_net.parameters(),net.parameters()):
            param_target.data.copy_(param_target.data * (1.0 - self.tau) + param.data * self.tau)
            
    def update(self,transition_dict):
        states = torch.tensor(transition_dict['states'], dtype=torch.float).to(self.device)
        laser_data = torch.tensor(transition_dict['laser_data'], dtype=torch.float).to(self.device)
        next_laser_data = torch.tensor(transition_dict['next_laser_data'], dtype=torch.float).to(self.device)
        # laser_features = self.laser_cnn(laser_data)
        # states = torch.cat([states, laser_features], dim=-1)
        # actions = torch.tensor(transition_dict['actions'], dtype=torch.float).view(-1, 1).to(self.device)
        actions = torch.tensor(transition_dict['actions'], dtype=torch.float).to(self.device)
        rewards = torch.tensor(transition_dict['rewards'], dtype=torch.float).view(-1, 1).to(self.device)
        next_states = torch.tensor(transition_dict['next_states'], dtype=torch.float).to(self.device)
        dones = torch.tensor(transition_dict['dones'], dtype=torch.float).view(-1, 1).to(self.device)
        
        # 确保拼接之前的维度匹配
        # print(f"states shape: {states.shape}, actions shape: {actions.shape}")
    
        #更新两个Q网络
        td_target=self.calc_target(rewards,next_states,dones,next_laser_data)
        #Q网络输出值和目标值的均方差
        critic_1_loss=torch.mean(F.mse_loss(self.critic_1(states,actions,laser_data),td_target.detach()))
        critic_2_loss=torch.mean(F.mse_loss(self.critic_2(states,actions,laser_data),td_target.detach()))
        self.critic_1_optimizer.zero_grad()
        critic_1_loss.backward()
        self.critic_1_optimizer.step()
        self.critic_2_optimizer.zero_grad()
        critic_2_loss.backward()
        self.critic_2_optimizer.step()
        
        #更新策略网络
        new_actions, log_prob=self.actor(states,laser_data)
        entropy= -log_prob
        q1_value=self.critic_1(states,new_actions,laser_data)
        q2_value=self.critic_2(states,new_actions,laser_data)
        #最大化价值，所以误差为价值函数加负号
        actor_loss=torch.mean(-self.log_alpha.exp() * entropy - torch.min(q1_value,q2_value))
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()
        
        target_q = torch.min(q1_value, q2_value)
        av_q = torch.mean(target_q)


        #更新alpha值
        #利用梯度下降自动调整熵正则项
        alpha_loss=torch.mean((entropy - self.target_entropy).detach() *self.log_alpha.exp())
        self.log_alpha_optimizer.zero_grad()
        alpha_loss.backward()
        self.log_alpha_optimizer.step()
        
        #软更新目标网络
        self.soft_update(self.critic_1,self.target_critic_1)
        self.soft_update(self.critic_2,self.target_critic_2)

        return alpha_loss, av_q

    def save_model(self, base_path, scenario):
        self.actor.save_checkpoint(os.path.join(base_path, f"leader_agent_actor_{scenario}.pth"))
        self.critic_1.save_checkpoint(os.path.join(base_path, f"leader_agent_critic1_{scenario}.pth"))
        self.critic_2.save_checkpoint(os.path.join(base_path, f"leader_agent_critic2_{scenario}.pth"))
        self.target_critic_1.save_checkpoint(os.path.join(base_path, f"leader_agent_target_critic1_{scenario}.pth"))
        self.target_critic_2.save_checkpoint(os.path.join(base_path, f"leader_agent_target_critic2_{scenario}.pth"))

    def load_model(self, base_path, scenario):
        self.actor.load_checkpoint(os.path.join(base_path, f"leader_agent_actor_{scenario}.pth"))
        self.critic_1.load_checkpoint(os.path.join(base_path, f"leader_agent_critic1_{scenario}.pth"))
        self.critic_2.load_checkpoint(os.path.join(base_path, f"leader_agent_critic2_{scenario}.pth"))
        self.target_critic_1.load_checkpoint(os.path.join(base_path, f"leader_agent_target_critic1_{scenario}.pth"))
        self.target_critic_2.load_checkpoint(os.path.join(base_path, f"leader_agent_target_critic2_{scenario}.pth"))


 