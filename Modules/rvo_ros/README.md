# 双向速度障碍物ROS库

## 介绍

这个ROS包是基于ORCA库（[库链接](http://gamma.cs.unc.edu/RVO2/)）衍生的。

![](https://github.com/hanruihua/rvo_ros/blob/master/simulation/rvo_sim.gif)

## 环境

- Ubuntu 18.04
- ROS Melodic

## 安装

> git clone https://github.com/hanruihua/rvo_ros.git  
> cd ~/catkin_ws  
> catkin_make  

## 设置环境参数

> export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/devel/lib

请将此行写入.bashrc或.zshrc文件中

## 使用方法

> rosrun rvo_ros rvo_node args

args：初始化点的坐标，默认0,1 0,2 ... 0,10

例如：

> rosrun rvo_ros rvo_node 0 1 0 2 0 3 

## 与仿真测试

<!-- > roslaunch rvo_ros rvo_gazebo_agent.launch -->
> roslaunch prometheus_gazebo rvo_gazebo_agent.launch

**注意**：使用服务来设置模型和目标。

## 服务

> rosrun rvo_ros set_goals_client

- 参数：
    - model：
        - "default"：为代理指定一系列点作为目标。目标的数量应与代理的数量相同：1 1 2 3 4 2。
        - "random"：沿x和y轴随机分配目标，仅限数量：min_x, max_x, min_y, max_y。
        - "circle"：以圆形分配目标：circle_point_x, circle_point_y, radius, flag。flag用于设置反向模式

- 示例：
    >rosrun rvo_ros set_goals_client default 1 1 1 4 4 4 4 1  
    >rosrun rvo_ros set_goals_client random 0 5 1 4   
    >rosrun rvo_ros set_goals_client circle 4 4 4 0 

## 主题

- 订阅的主题

/rvo/model_states ([gazebo_msgs/ModelStates](http://docs.ros.org/jade/api/gazebo_msgs/html/msg/ModelStates.html))

**注意**：为避免模型混淆，只有类似'agent+num'样式的模型名称，例如agent1、agent2，可以被视为代理模型。

- 发布的主题

/rvo_vel ([gazebo_msgs/WorldStates](http://docs.ros.org/jade/api/gazebo_msgs/html/msg/WorldState.html))

**注意**：仅将每个代理从rvo计算得出的x、y方向的速度设置在WorldStates的twist部分。

## 作者

**Han** - [Han](https://github.com/hanruihua)  

## 许可证

该项目根据MIT许可证授权

## 概览（[[论文]](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.162.265&rep=rep1&type=pdf)）

多机器人双向n体碰撞避免方法，多个移动机器人在共同的工作空间中移动时需要避免相互碰撞。

假设：

1. 每个机器人假设具有简单形状（圆形或凸多边形），在二维工作空间中移动。
2. 机器人是全向的，即它可以朝任何方向移动，因此每个机器人的控制输入简单地由二维速度向量给出。
3. 每个机器人具有完美的感应能力，并能够准确推断出障碍物和其他机器人的确切形状、位置和速度。

优点：

1. 机器人之间不需要通信。
2. 可以处理静态障碍物。
3. 可以保证在拥挤的工作空间中大量机器人的局部无碰撞运动。

限制：

1. 在现实世界中，完美感应

的假设难以执行，因为存在不确定性。
2. 构造复杂模型需要太多参数。