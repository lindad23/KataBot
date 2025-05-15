import sys
import numpy as np
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
import pyqtgraph.opengl as gl
import pyqtgraph as pg

# 创建应用
app = QApplication(sys.argv)
print(sys.argv)
view = gl.GLViewWidget()
view.setWindowTitle('3D Coordinate Axes with Moving Point')
view.setGeometry(100, 100, 800, 600)
view.opts['distance'] = 20  # 初始视角距离
view.show()

# 添加坐标轴
axis_length = 10

# X轴（红色）
x_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [axis_length, 0, 0]]), color=(1, 0, 0, 1), width=2, antialias=True)
# Y轴（绿色）
y_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, axis_length, 0]]), color=(0, 1, 0, 1), width=2, antialias=True)
# Z轴（蓝色）
z_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, axis_length]]), color=(0, 0, 1, 1), width=2, antialias=True)

view.addItem(x_axis)
view.addItem(y_axis)
view.addItem(z_axis)

# 添加运动点
point_pos = np.array([[0, 0, 0]])
moving_point = gl.GLScatterPlotItem(pos=point_pos, color=(1, 1, 0, 1), size=10)
view.addItem(moving_point)

# 动态更新点位置（沿着一个螺旋轨迹）
t = 0.0
def update():
    global t
    t += 0.05
    x = 5 * np.cos(t)
    y = 5 * np.sin(t)
    z = t * 0.5
    moving_point.setData(pos=np.array([[x, y, z]]))

# 每 30ms 更新一次位置
timer = QTimer()
timer.timeout.connect(update)
timer.start(30)

visible = True
# 每秒切换可见状态
def toggle_visibility():
    global visible
    if visible:
        view.removeItem(moving_point)
    else:
        view.addItem(moving_point)
    visible = not visible

visibility_timer = QTimer()
visibility_timer.timeout.connect(toggle_visibility)
visibility_timer.start(1000)

# 运行应用
sys.exit(app.exec_())
