import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# 初始化图像
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
line, = ax.plot([], [], [], 'b-')  # 初始化线
point, = ax.plot([], [], [], 'ro')  # 初始化点

# 设定坐标轴范围
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(-10, 10)

# 初始化函数
def init():
    line.set_data([], [])
    line.set_3d_properties([])
    point.set_data([], [])
    point.set_3d_properties([])
    return line, point

# 更新函数
x_data, y_data, z_data = [], [], []
def update(frame):
    t = frame * 0.1
    x, y, z = np.sin(t), np.cos(t), t * 0.2
    x_data.append(x)
    y_data.append(y)
    z_data.append(z)
    
    line.set_data(x_data, y_data)
    line.set_3d_properties(z_data)
    point.set_data([x], [y])
    point.set_3d_properties([z])
    return line, point

ani = FuncAnimation(fig, update, init_func=init, frames=range(200), interval=50, blit=False)
plt.show()
