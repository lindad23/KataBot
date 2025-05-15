import sys
import time
import numpy as np
from threading import Thread
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
import pyqtgraph.opengl as gl  # 导包必须在PyQt5后

class Render3D:
    def __init__(self, dt=0.03, window_title="Render 3D"):
        self.dt = dt  # second
        self.window_title = window_title
        self.create_app()
    
    def create_app(self):
        print("GG1")
        self.app = QApplication([])
        print("GG2")
        self.view = gl.GLViewWidget()
        print("GG3")
        self.view.setWindowTitle(self.window_title)
        self.view.setGeometry(100, 100, 800, 600)  # window width=800, height=600
        self.view.opts['distance'] = 20  # window camera distance
        self.view.show()
        self.app.exec_()
    
    def run(self):
        while True:
            time.sleep(self.dt)

if __name__ == '__main__':
    render3d = Render3D()
    render3d.run()