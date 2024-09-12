#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import time
import os
import numpy as np

# 切换为非交互的后端 'Agg'
import matplotlib
matplotlib.use('Agg')

# 存储时间和最小距离数据
time_data_1 = []
time_data_2 = []
time_data_3 = []
uav1_min_distances = []  # 存储 UAV1 的最小距离
uav2_min_distances = []  # 存储 UAV2 的最小距离
uav3_min_distances = []  # 存储 UAV3 的最小距离
start_time = time.time()

# 全局变量用于保存接收到的最新话题数据
uav1_distance_to_2 = None
uav1_distance_to_3 = None
uav2_distance_to_1 = None
uav2_distance_to_3 = None
uav3_distance_to_1 = None
uav3_distance_to_2 = None

# 固定纵坐标的范围
y_limit = [0, 25]   # Y轴代表接收到的值，范围固定为0到25
x_limit = [0, 70]   # X轴固定为0到80秒
T_therahold = [10, 85]

# 初始化绘图
fig, ax = plt.subplots(figsize=(8, 6))  # 单个绘图
# ax.set_title('UAV Min Distances')

# 设置y=2.0的标识线
ax.axhline(y=2.0, color='black', linestyle='--')
ax.text(x=0, y=2.0, s='2.0', color='black', verticalalignment='bottom', horizontalalignment='right')

# 更新 UAV1 的图表
def update_plot_uav1(event):
    global time_data_1, uav1_min_distances
    
    # 获取当前时间（相对开始时间）
    current_time = time.time() - start_time

    # 确保 UAV1 的两个话题数据都已接收到
    if uav1_distance_to_2 is not None and uav1_distance_to_3 is not None:
        # 计算 UAV1 的较小距离
        uav1_min_distance = min(uav1_distance_to_2, uav1_distance_to_3)

        # 添加时间和距离数据，检查时间是否在 10 到 80 秒之间
        if T_therahold[0] <= current_time <= T_therahold[1]:
            time_data_1.append(current_time - 10)  # 时间从 0 开始（减去10秒）
            uav1_min_distances.append(uav1_min_distance)

        # 清除之前的数据并重新绘制
        ax.cla()
        ax.axhline(y=2.0, color='black', linestyle='--')
        ax.text(x=0, y=2.0, s='2.0', color='black', verticalalignment='bottom', horizontalalignment='right')

        # 绘制所有 UAV 的数据
        ax.plot(time_data_1, uav1_min_distances, label='A1', color='red')
        if time_data_2 and uav2_min_distances:
            ax.plot(time_data_2, uav2_min_distances, label='A2', color='green')
        if time_data_3 and uav3_min_distances:
            ax.plot(time_data_3, uav3_min_distances, label='A3', color='yellow')

        ax.set_xlim(x_limit)  # 固定X轴范围为0到80秒
        ax.set_ylim(y_limit)
        ax.set_xlabel('Time/s')
        ax.set_ylabel('Minimum Distance')
        ax.legend()

# 更新 UAV2 的图表
def update_plot_uav2(event):
    global time_data_2, uav2_min_distances
    
    # 获取当前时间（相对开始时间）
    current_time = time.time() - start_time

    # 确保 UAV2 的两个话题数据都已接收到
    if uav2_distance_to_1 is not None and uav2_distance_to_3 is not None:
        # 计算 UAV2 的较小距离
        uav2_min_distance = min(uav2_distance_to_1, uav2_distance_to_3)

        # 添加时间和距离数据，检查时间是否在 10 到 80 秒之间
        if T_therahold[0] <= current_time <= T_therahold[1]:
            time_data_2.append(current_time - 10)  # 时间从 0 开始（减去10秒）
            uav2_min_distances.append(uav2_min_distance)

        # 清除之前的数据并重新绘制
        ax.cla()
        ax.axhline(y=2.0, color='black', linestyle='--')
        ax.text(x=0, y=2.0, s='2.0', color='black', verticalalignment='bottom', horizontalalignment='right')

        # 绘制所有 UAV 的数据
        ax.plot(time_data_1, uav1_min_distances, label='A1', color='red')
        if time_data_2 and uav2_min_distances:
            ax.plot(time_data_2, uav2_min_distances, label='A2', color='green')
        if time_data_3 and uav3_min_distances:
            ax.plot(time_data_3, uav3_min_distances, label='A3', color='yellow')

        ax.set_xlim(x_limit)  # 固定X轴范围为0到80秒
        ax.set_ylim(y_limit)
        ax.set_xlabel('Time/s')
        ax.set_ylabel('Minimum Distance')
        ax.legend()

# 更新 UAV3 的图表
def update_plot_uav3(event):
    global time_data_3, uav3_min_distances
    
    # 获取当前时间（相对开始时间）
    current_time = time.time() - start_time

    # 确保 UAV3 的两个话题数据都已接收到
    if uav3_distance_to_1 is not None and uav3_distance_to_2 is not None:
        # 计算 UAV3 的较小距离
        uav3_min_distance = min(uav3_distance_to_1, uav3_distance_to_2)

        # 添加时间和距离数据，检查时间是否在 10 到 80 秒之间
        if T_therahold[0] <= current_time <= T_therahold[1]:
            time_data_3.append(current_time - 10)  # 时间从 0 开始（减去10秒）
            uav3_min_distances.append(uav3_min_distance)

        # 清除之前的数据并重新绘制
        ax.cla()
        ax.axhline(y=2.0, color='black', linestyle='--')
        ax.text(x=0, y=2.0, s='2.0', color='black', verticalalignment='bottom', horizontalalignment='right')

        # 绘制所有 UAV 的数据
        ax.plot(time_data_1, uav1_min_distances, label='A1', color='red')
        if time_data_2 and uav2_min_distances:
            ax.plot(time_data_2, uav2_min_distances, label='A2', color='green')
        if time_data_3 and uav3_min_distances:
            ax.plot(time_data_3, uav3_min_distances, label='A3', color='yellow')

        ax.set_xlim(x_limit)  # 固定X轴范围为0到80秒
        ax.set_ylim(y_limit)
        ax.set_xlabel('Time/s')
        ax.set_ylabel('Minimum Distance')
        ax.legend()

# 各个 UAV 的话题回调函数
def callback_uav1_distance_to_2(data):
    global uav1_distance_to_2
    uav1_distance_to_2 = data.data

def callback_uav1_distance_to_3(data):
    global uav1_distance_to_3
    uav1_distance_to_3 = data.data

def callback_uav2_distance_to_1(data):
    global uav2_distance_to_1
    uav2_distance_to_1 = data.data

def callback_uav2_distance_to_3(data):
    global uav2_distance_to_3
    uav2_distance_to_3 = data.data

def callback_uav3_distance_to_1(data):
    global uav3_distance_to_1
    uav3_distance_to_1 = data.data

def callback_uav3_distance_to_2(data):
    global uav3_distance_to_2
    uav3_distance_to_2 = data.data

# 保存图像
def save_plot():
    plt.ioff()

    save_dir = "images"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    save_path = os.path.join(save_dir, "min_distance_exp.svg")
    print(f"Saving plot to: {save_path}")

    # 保存当前绘制的图像
    plt.savefig(save_path, format='svg')
    print("Plot saved successfully!")

# 主监听函数
def listener():
    # 初始化ROS节点
    rospy.init_node('distance_plotter', anonymous=True)

    # 订阅 /uav1 的两个话题
    rospy.Subscriber("/uav1/distance_to_2", Float32, callback_uav1_distance_to_2)
    rospy.Subscriber("/uav1/distance_to_3", Float32, callback_uav1_distance_to_3)

    # 订阅 /uav2 的两个话题
    rospy.Subscriber("/uav2/distance_to_1", Float32, callback_uav2_distance_to_1)
    rospy.Subscriber("/uav2/distance_to_3", Float32, callback_uav2_distance_to_3)

    # 订阅 /uav3 的两个话题
    rospy.Subscriber("/uav3/distance_to_1", Float32, callback_uav3_distance_to_1)
    rospy.Subscriber("/uav3/distance_to_2", Float32, callback_uav3_distance_to_2)

    # 设置三个定时器，分别更新三个 UAV 的绘图
    rospy.Timer(rospy.Duration(0.4), update_plot_uav1)  # 每隔 500ms 更新 UAV1 的图表
    rospy.Timer(rospy.Duration(0.4), update_plot_uav2)  # 每隔 500ms 更新 UAV2 的图表
    rospy.Timer(rospy.Duration(0.4), update_plot_uav3)  # 每隔 500ms 更新 UAV3 的图表

    # 保持ROS节点运行
    rospy.spin()

    # 保存图表
    save_plot()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
