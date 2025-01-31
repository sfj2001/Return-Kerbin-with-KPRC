import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 读取 CSV 文件E:\KRPC_dode_with_gpt\再入段控制\回收程序\data_log.csv
file_path = "./src/data_log.csv"  # 替换为你的 CSV 文件路径
df = pd.read_csv(file_path)

# 假设第一列是无用数据，跳过它
x = df.iloc[:, 1]  # 第二列作为 X 坐标
y = df.iloc[:, 2]  # 第三列作为 Y 坐标
z = df.iloc[:, 3]  # 第四列作为 Z 坐标

# 定义球的参数
radius = 600_000  # 星球半径
num_points = 50  # 球面分辨率

# 生成球的坐标 (球坐标系转换为直角坐标系)
theta = np.linspace(0, np.pi, num_points)  # 纬度
phi = np.linspace(0, 2 * np.pi, num_points)  # 经度
theta, phi = np.meshgrid(theta, phi)

x_sphere = radius * np.sin(theta) * np.cos(phi)
y_sphere = radius * np.sin(theta) * np.sin(phi)
z_sphere = radius * np.cos(theta)

# 创建 3D 绘图
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# # 绘制星球表面
# ax.plot_surface(x_sphere, y_sphere, z_sphere, color='c', alpha=0.4, edgecolor='k')

# 绘制飞行器轨迹
ax.plot(x, y, z, color='r', linewidth=2, label='Trajectory')

# 绘制轨迹起点和终点的突出显示
ax.scatter(x.iloc[0], y.iloc[0], z.iloc[0], color='g', s=100, label='Start Point', marker='o')
ax.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], color='b', s=100, label='End Point', marker='x')

# 设置轴标签
ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")
ax.set_title("3D Trajectory with Planet")

# 设置坐标轴范围
ax.set_xlim([-radius-10000, radius+10000])
ax.set_ylim([-radius-10000, radius+10000])
ax.set_zlim([-radius-10000, radius+10000])

# 添加图例
ax.legend()

# 显示图形
plt.show()