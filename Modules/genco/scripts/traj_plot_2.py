#!/usr/bin/env python
import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np
import os
from matplotlib.colors import LinearSegmentedColormap

# 替换为你的rosbag文件路径
bag_file = '/home/reek/Prometheus/Modules/genco/rosbag/2024-09-09-17-03-02.bag'

LINEWIDTH = 5
AX_WIDTH = 2

# 初始化用于存储轨迹点的列表
x_data_uav1 = []
y_data_uav1 = []
z_data_uav1 = []

x_data_uav2 = []
y_data_uav2 = []
z_data_uav2 = []

x_data_uav3 = []
y_data_uav3 = []
z_data_uav3 = []

# 设定一个最大时间阈值，80秒
time_threshold = 80.0

# 读取rosbag文件
with rosbag.Bag(bag_file, 'r') as bag:
    # 记录初始时间，用于计算相对时间
    initial_time = None

    # 遍历 /uav1, /uav2 和 /uav3 的 drone_trajectory_exp 话题
    for topic, msg, t in bag.read_messages(topics=['/uav1/prometheus/drone_trajectory_exp', 
                                                   '/uav2/prometheus/drone_trajectory_exp',
                                                   '/uav3/prometheus/drone_trajectory_exp']):
        # 如果初始时间尚未设置，设置为第一条消息的时间
        if initial_time is None:
            initial_time = t.to_sec()

        # 计算相对时间
        relative_time = t.to_sec() - initial_time

        # 只处理时间在 80 秒内的数据
        if relative_time <= time_threshold:
            # 遍历每个PoseStamped，提取位置数据
            if topic == '/uav1/prometheus/drone_trajectory_exp':
                for pose_stamped in msg.poses:
                    x_data_uav1.append(pose_stamped.pose.position.x)
                    y_data_uav1.append(pose_stamped.pose.position.y)
                    z_data_uav1.append(pose_stamped.pose.position.z)

            elif topic == '/uav2/prometheus/drone_trajectory_exp':
                for pose_stamped in msg.poses:
                    x_data_uav2.append(pose_stamped.pose.position.x)
                    y_data_uav2.append(pose_stamped.pose.position.y)
                    z_data_uav2.append(pose_stamped.pose.position.z)

            elif topic == '/uav3/prometheus/drone_trajectory_exp':
                for pose_stamped in msg.poses:
                    x_data_uav3.append(pose_stamped.pose.position.x)
                    y_data_uav3.append(pose_stamped.pose.position.y)
                    z_data_uav3.append(pose_stamped.pose.position.z)
        else:
            # 如果时间超过80秒，跳出循环
            break

# 将数据转换为 NumPy 数组以便于操作
x_data_uav1 = np.array(x_data_uav1)
y_data_uav1 = np.array(y_data_uav1)
z_data_uav1 = np.array(z_data_uav1)

x_data_uav2 = np.array(x_data_uav2)
y_data_uav2 = np.array(y_data_uav2)
z_data_uav2 = np.array(z_data_uav2)

x_data_uav3 = np.array(x_data_uav3)
y_data_uav3 = np.array(y_data_uav3)
z_data_uav3 = np.array(z_data_uav3)

# 创建颜色渐变的函数
def color_gradient(ax, x_data, y_data, z_data, color_start, color_end):
    points = np.array([x_data, y_data, z_data]).T.reshape(-1, 1, 3)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # 创建从 color_start 到 color_end 的线性渐变
    cmap = LinearSegmentedColormap.from_list('custom_cmap', [color_start, color_end])
    colors = np.linspace(0, 1, len(x_data) - 1)

    lc = Line3DCollection(segments, cmap=cmap, norm=plt.Normalize(0, 1))
    lc.set_array(colors)
    lc.set_linewidth(LINEWIDTH)
    ax.add_collection(lc)

# 创建一个3D图
fig = plt.figure(figsize=(16, 9))  # 宽度为16英寸，高度为9英寸
ax = fig.add_subplot(111, projection='3d')

# 绘制 UAV1 轨迹，红色渐变到深红
color_gradient(ax, x_data_uav1, y_data_uav1, z_data_uav1, color_start='lightcoral', color_end='darkred')

# 绘制 UAV2 轨迹，绿色渐变到深绿
color_gradient(ax, x_data_uav2, y_data_uav2, z_data_uav2, color_start='lightgreen', color_end='darkgreen')

# 绘制 UAV3 轨迹，黄色渐变到橙色
color_gradient(ax, x_data_uav3, y_data_uav3, z_data_uav3, color_start='lightyellow', color_end='darkorange')

# 设置X, Y, Z轴的范围
ax.set_xlim([0, 10])
ax.set_ylim([-20, 0])
ax.set_zlim([0, 10])

ax.view_init(elev=30, azim=210)  # 仰角为30度，方位角为45度


# 修改坐标轴刻度标签的字体大小
ax.tick_params(axis='both', which='major', labelsize=26)

# 修改坐标轴的粗细
ax.xaxis.line.set_linewidth(AX_WIDTH)
ax.yaxis.line.set_linewidth(AX_WIDTH)
ax.zaxis.line.set_linewidth(AX_WIDTH)

# 设置X, Y, Z轴的刻度间隔
ax.set_xticks(np.arange(0, 11, 2))
ax.set_yticks(np.arange(-20, 1, 5))
ax.set_zticks(np.arange(0, 11, 2))

# 保存图像为PDF
output_dir = "output_images"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

pdf_path = os.path.join(output_dir, "uav_trajectory_3.pdf")
plt.savefig(pdf_path, format='pdf')
print(f"Plot saved as {pdf_path}")

# 如果需要显示图形，取消注释下面这一行
# plt.show()
