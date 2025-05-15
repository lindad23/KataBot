import sys
import time
import numpy as np
from threading import Thread
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
import pyqtgraph.opengl as gl  # 导包必须在PyQt5后

class Object3D:
    items: list
    visibles: bool
    def update_item_data(self, **kwargs): ...
    def update_view(self, view): ...

class Axis(Object3D):
    def __init__(self, rotation: np.ndarray, translation: np.ndarray, axis_length: float = 10.):
        self.rotation, self.translation = rotation, translation
        self.axis_length = axis_length

        self.origin = gl.GLScatterPlotItem()
        self.colors = np.array([(1,0,0,1), (0,1,0,1), (0,0,1,1)])
        self.axes = []
        for i in range(3):
            self.axes.append(gl.GLLinePlotItem())
        self.items = [self.origin, *self.axes]
        self.visibles = [True] * 4
        self.update_item_data()
    
    def update_item_data(self, rotation: np.ndarray = None, translation: np.ndarray = None):
        if rotation is not None:
            self.rotation = rotation
        if translation is not None:
            self.translation = translation

        base_lines = np.array([(1,0,0), (0,1,0), (0,0,1)]) * self.axis_length @ self.rotation.T + self.translation.reshape(1, -1)
        self.axes = []
        for i in range(3):
            self.axes.append(gl.GLLinePlotItem(pos=base_lines[i], color=self.colors[i], width=2, antialias=True))
        self.origin.setData(pos=self.translation)
    
    def update_view(self, view: gl.GLViewWidget):
        for vis, item in zip(self.visibles, self.items):
            if vis and item not in view.items:
                view.addItem(item)
            if not vis and item in view.items:
                view.removeItem(item)

class Point(Object3D):
    def __init__(self, position, color=(1,1,1,1), size=10):
        self.pos = gl.GLScatterPlotItem(pos=position, color=color, size=size)


class Render3D:
    def __init__(self, dt=0.03, window_title="Render 3D"):
        self.dt = dt  # second
        self.window_title = window_title
        self.objs: set[Object3D] = set()
        self.start_thread()
    
    def create_app(self):
        self.app = QApplication(sys.argv)
        self.view = gl.GLViewWidget()
        self.view.setWindowTitle(self.window_title)
        self.view.setGeometry(100, 100, 800, 600)  # window width=800, height=600
        self.view.opts['distance'] = 20  # window camera distance
        self.view.show()
        self.app.exec_()
    
    def add_obj(self, obj: Object3D):
        self.objs.add(obj)
    
    def run(self):
        self.create_app()
        print("GG1")
        while True:
            print("GG2")
            print(self.objs)
            for obj in self.objs:
                obj.update_view(self.view)
            time.sleep(self.dt)
    
    def start_thread(self):
        print("GG")
        self.thread = Thread(target=self.run, daemon=True)
        self.thread.start()

if __name__ == '__main__':
    render3d = Render3D()
    axis = Axis(rotation=np.eye(3, 3), translation=np.zeros(3))
    render3d.add_obj(axis)
    while True:
        ...
