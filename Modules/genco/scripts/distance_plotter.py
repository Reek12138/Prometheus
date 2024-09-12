#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import time
import os
import numpy as np
from scipy.interpolate import interp1d

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
y_limit = [0, 21]   # Y轴代表接收到的值，范围固定为0到25
x_limit = [0, 65]   # X轴固定为0到80秒
T_therahold = [10, 80]

# 初始化绘图
fig, ax = plt.subplots(figsize=(8, 6))  # 单个绘图
# ax.set_title('UAV Min Distances')
# 调整图例字体大小
# ax.legend(fontsize=14)  # 将字体大小设置为14
# 调整图例字体大小并加粗
ax.legend(fontsize=14, loc='upper right', prop={'weight': 'bold'})


# 调整坐标轴刻度标签字体大小
ax.tick_params(axis='both', which='major', labelsize=14)  # 将字体大小设置为16

ax.set_yticks(np.arange(0, 26, 5))
yticks = ax.get_yticks()  # 获取当前的 y 轴刻度
if 2 not in yticks:  # 如果 2 不在当前的刻度中，手动添加
    yticks = np.append(yticks, 2)
ax.set_yticks(yticks)  # 设置新的 y 轴刻度
# 调整X轴和Y轴标签的字体大小
ax.set_xlabel('Time/s', fontsize=20)  # X轴标签字体大小为16
ax.set_ylabel('Minimum Distance', fontsize=20)  # Y轴标签字体大小为16

# 调整坐标轴刻度标签的字体大小和刻度线粗细
ax.tick_params(axis='both', which='major', labelsize=20, width=4, length=10)  # 刻度标签大小14，刻度线宽度2
# 调整图例字体大小并加粗

# 调整轴线的粗细
for spine in ax.spines.values():
    spine.set_linewidth(3.5)  # 将轴线宽度设置为2

# 调整布局，避免标签被遮挡
plt.tight_layout()


# 绘制初始的线条
line_uav3, = ax.plot([], [], label='A1', color='red', linestyle=(0, (8,5)), linewidth=5)
line_uav2, = ax.plot([], [], label='A2', color='green', linestyle=(0, (3, 1)), linewidth=5)
line_uav1, = ax.plot([], [], label='A3', color='yellow', linestyle='-.', linewidth=5)

# 设置y=2.0的标识线
ax.axhline(y=2.0, color='blue', linestyle=':', linewidth=3)
# ax.text(x=0, y=2.0, s='2 ', color='black', verticalalignment='bottom', horizontalalignment='right', fontsize=20)

# 固定X轴和Y轴的范围
ax.set_xlim(x_limit)
ax.set_ylim(y_limit)
ax.set_xlabel('Time/s')
ax.set_ylabel('Minimum Distance')
ax.legend()  # 图例只绘制一次

# 滑动平均平滑函数
def moving_average(data, window_size=15):
    """应用滑动平均对数据进行平滑"""
    if len(data) < window_size:
        return data  # 如果数据点少于窗口大小，则返回原始数据
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

# 插值函数
def interpolate_data(x_data, y_data, num_points=500):
    if len(x_data) < 2:  # 需要至少两个数据点进行插值
        return x_data, y_data
    interp_func = interp1d(x_data, y_data, kind='linear', fill_value="extrapolate")
    x_new = np.linspace(min(x_data), max(x_data), num=num_points)
    y_new = interp_func(x_new)
    return x_new, y_new

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

        # 插值处理
        time_data_interp, uav1_min_distances_interp = interpolate_data(time_data_1, uav1_min_distances)

        # 对插值后的数据进行平滑处理
        uav1_min_distances_smoothed = moving_average(uav1_min_distances_interp, window_size=15)

        # 更新线条数据
        line_uav1.set_data(time_data_interp[:len(uav1_min_distances_smoothed)], uav1_min_distances_smoothed)
        fig.canvas.draw()  # 刷新图形

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
            time_data_2.append(current_time - 10)
            uav2_min_distances.append(uav2_min_distance)

        # 插值处理
        time_data_interp, uav2_min_distances_interp = interpolate_data(time_data_2, uav2_min_distances)

        # 对插值后的数据进行平滑处理
        uav2_min_distances_smoothed = moving_average(uav2_min_distances_interp, window_size=15)

        # 更新线条数据
        line_uav2.set_data(time_data_interp[:len(uav2_min_distances_smoothed)], uav2_min_distances_smoothed)
        fig.canvas.draw()

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
            time_data_3.append(current_time - 10)
            uav3_min_distances.append(uav3_min_distance)

        # 插值处理
        time_data_interp, uav3_min_distances_interp = interpolate_data(time_data_3, uav3_min_distances)

        # 对插值后的数据进行平滑处理
        uav3_min_distances_smoothed = moving_average(uav3_min_distances_interp, window_size=15)

        # 更新线条数据
        line_uav3.set_data(time_data_interp[:len(uav3_min_distances_smoothed)], uav3_min_distances_smoothed)
        fig.canvas.draw()

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

    save_path = os.path.join(save_dir, "min_distance_over_time.pdf")
    print(f"Saving plot to: {save_path}")

    # 保存当前绘制的图像
    plt.savefig(save_path, format='pdf')
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
    rospy.Timer(rospy.Duration(0.2), update_plot_uav1)  # 更高频率的更新 UAV1 的图表
    rospy.Timer(rospy.Duration(0.2), update_plot_uav2)  # 更高频率的更新 UAV2 的图表
    rospy.Timer(rospy.Duration(0.2), update_plot_uav3)  # 更高频率的更新 UAV3 的图表

    # 保持ROS节点运行
    rospy.spin()

    # 保存图表
    save_plot()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass









