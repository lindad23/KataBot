import sys
import numpy as np
from PyQt5.QtWidgets import QApplication
import pyqtgraph.opengl as gl
from pyqtgraph import Vector

class MyQtAPP:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.view = gl.GLViewWidget()
        self.view.setWindowTitle("qtgraphe test")
        # Use self.view.opts to config default camera
        self.view.opts['distance'] = 20  # after add option can move axis
        self.view.opts['center'] = Vector(0, 0, 2)  # center point
        self.view.setGeometry(0, 0, 800, 600)  # set view width and height
        self.view.show()
    
    def add_axis(self):
        axis_len = 10
        axis_width = 2
        axis = [
            gl.GLLinePlotItem(pos=np.array([[0,0,0], [axis_len,0,0]]), color=(1,0,0,1), width=axis_width, antialias=True),
            gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,axis_len,0]]), color=(0,1,0,1), width=axis_width, antialias=True),
            gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,0,axis_len]]), color=(0,0,1,1), width=axis_width, antialias=True),
        ]
        for a in axis:
            self.view.addItem(a)
    
    def add_grid(self):
        def add_axis_plane(rotate):
            grid = gl.GLGridItem()
            grid.setSpacing(1, 1)
            grid.setSize(10, 10)
            grid.translate(5, 5, 0)
            grid.rotate(*rotate)
            self.view.addItem(grid)
        add_axis_plane((0, 1, 0, 0))
        add_axis_plane((90, 1, 0, 0))
        add_axis_plane((-90, 0, 1, 0))
    
    def add_origin_marker(self):
        origin = gl.GLScatterPlotItem(pos=(0,0,0), color=(1,1,1,1), size=10)
        self.view.addItem(origin)
    
    def run(self):
        self.add_axis()
        self.add_grid()
        self.add_origin_marker()
        sys.exit(self.app.exec_())


if __name__ == '__main__':
    my_qtapp = MyQtAPP()
    my_qtapp.run()

