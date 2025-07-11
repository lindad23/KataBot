import cv2
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QProgressBar, QFileDialog, QMessageBox, QGridLayout  # 新增 QGridLayout 导入
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import threading
import time
import os
import yaml
import socket
import sys
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from demo5_gui.scripts.constants import SURFACE_CLASSES

class PlayVideoThread(QThread):
    frame_signal = pyqtSignal(object)
    time_signal = pyqtSignal(str)
    progress_signal = pyqtSignal(int)
    debug_signal = pyqtSignal(list)
    def __init__(self, gui):
        super().__init__()
        self.gui = gui
        self.thread_running = True
        # 用于debug的时间点和数据
        self.debug_times = [1, 3, 5, 8, 10]
        self.debug_data = [
            [["top", 1], ["botton", 2], ["right", 1]],
            [["top", 2], ["botton", 5], ["right", 6]],
            [["top", 3], ["botton", 5], ["right", 6]],
            [["top", 4], ["botton", 7], ["right", 8]],
            [["top", 5], ["botton", 9], ["right", 10]]
        ]
        self.debug_index = 0
        self.start_time = None
    def run(self):
        self.start_time = time.time()
        while self.thread_running:
            if self.start_time is None:
                self.start_time = time.time()
            if self.gui.current_frame is not None:
                self.gui.current_frame_index += 1
                # 更新进度条
                self.progress_signal.emit(self.gui.current_frame_index)
                # 更新时间显示
                elapsed_seconds = time.time() - self.start_time
                current_time_str = time.strftime('%M:%S', time.gmtime(elapsed_seconds))
                self.time_signal.emit(current_time_str)
                # 检查是否到达debug时间点
                if self.debug_index < len(self.debug_times) and elapsed_seconds >= self.debug_times[self.debug_index]:
                    self.debug_signal.emit(self.debug_data[self.debug_index])
                    self.gui.screw_info.append(self.debug_data[self.debug_index])
                    self.debug_index += 1
                # 发送当前帧信号
                self.frame_signal.emit(self.gui.current_frame)
                # 录制视频
                if self.gui.recording:
                    if self.gui.out is None:
                        self.gui.init_video_writer()
                    self.gui.out.write(self.gui.current_frame)
            # 控制播放速度
            time.sleep(self.gui.frame_delay / 1000)
    def stop(self):
        self.thread_running = False
        self.wait()

class GUI(QWidget):
    def __init__(self, send_pause_command):
        super().__init__()
        self.send_pause_command = send_pause_command
        self.initUI()
        # 视频相关变量
        self.cap = None
        self.current_frame = None
        self.playing = False
        self.frame_delay = 40  # 默认播放速度，毫秒
        self.total_frames = 0
        self.current_frame_index = 0
        self.recording = False
        self.out = None
        self.screw_info = []
        # 线程变量
        self.play_thread = None
        self.save_thread = None

    def initUI(self):
        self.setWindowTitle("Camera Transform GUI")
        self.setGeometry(100, 100, 1500, 800)
        # 创建主布局
        main_layout = QHBoxLayout()
        # 视频播放区域
        video_layout = QVBoxLayout()
        self.canvas = QLabel(self)
        self.canvas.setFixedSize(1200, 700)
        self.canvas.setStyleSheet("background-color: black;")
        video_layout.addWidget(self.canvas)
        # 进度条和时间显示
        progress_layout = QHBoxLayout()
        self.time_label = QLabel("00:00", self)
        progress_layout.addWidget(self.time_label)
        self.progress_bar = QProgressBar(self)
        self.progress_bar.setOrientation(Qt.Horizontal)
        self.progress_bar.setFixedWidth(1000)
        progress_layout.addWidget(self.progress_bar)
        video_layout.addLayout(progress_layout)
        # 计算区域
        calculate_layout = QVBoxLayout()
        # 上部分六个框，3*2布局
        top_part = QGridLayout()
        self.names = ["top", "botton", "front", "back", "left", "right"]
        self.labels = {}
        self.max_crew = {name: 0 for name in self.names}
        # 定义字体大小
        font_size = 12
        # 定义边框样式
        border_width = 2
        border_color = "blue"
        border_radius = 10
        box_style = f"border: {border_width}px solid {border_color}; border-radius: {border_radius}px; padding: 10px;"
        for i, name in enumerate(self.names):
            row = i // 2
            col = i % 2
            frame = QWidget(self)
            frame.setStyleSheet(box_style)
            layout = QVBoxLayout(frame)
            name_label = QLabel(name, self)
            name_label.setAlignment(Qt.AlignCenter)
            name_label.setStyleSheet(f"font-size: {font_size}px;")
            layout.addWidget(name_label)
            crew_label = QLabel(f"have max crew:0", self)
            crew_label.setAlignment(Qt.AlignCenter)
            crew_label.setStyleSheet(f"font-size: {font_size}px;")
            layout.addWidget(crew_label)
            top_part.addWidget(frame, row, col)
            self.labels[name] = crew_label
        # 下部分按钮
        bottom_part = QVBoxLayout()
        self.start_record_button = QPushButton("开始录制", self)
        self.start_record_button.clicked.connect(self.start_recording)
        bottom_part.addWidget(self.start_record_button)
        self.stop_record_button = QPushButton("停止录制", self)
        self.stop_record_button.clicked.connect(self.stop_recording)
        bottom_part.addWidget(self.stop_record_button)
        self.save_button = QPushButton("保存视频", self)
        self.save_button.clicked.connect(self.save_video)
        bottom_part.addWidget(self.save_button)
        self.pause_button = QPushButton("暂停", self)
        self.pause_button.clicked.connect(self.pause_stream)
        bottom_part.addWidget(self.pause_button)
        calculate_layout.addLayout(top_part)
        calculate_layout.addLayout(bottom_part)
        main_layout.addLayout(video_layout)
        main_layout.addLayout(calculate_layout)
        self.setLayout(main_layout)

    def load_video_stream(self):
        self.playing = True
        self.play_thread = PlayVideoThread(self)
        self.play_thread.frame_signal.connect(self.display_frame)
        self.play_thread.time_signal.connect(self.update_time_label)
        self.play_thread.progress_signal.connect(self.update_progress_bar)
        self.play_thread.debug_signal.connect(self.calcu_crew)
        self.play_thread.start()

    def toggle_play(self):
        """切换播放/暂停状态"""
        if self.playing:
            self.playing = False
            if self.play_thread:
                self.play_thread.stop()
        else:
            self.playing = True
            self.play_thread = PlayVideoThread(self)
            self.play_thread.frame_signal.connect(self.display_frame)
            self.play_thread.time_signal.connect(self.update_time_label)
            self.play_thread.progress_signal.connect(self.update_progress_bar)
            self.play_thread.debug_signal.connect(self.calcu_crew)
            self.play_thread.start()

    def display_frame(self, frame):
        """显示当前帧"""
        if frame is not None:
            # 转换BGR为RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # 调整帧大小以适应Canvas
            height, width, _ = rgb_frame.shape
            canvas_width, canvas_height = 1200, 700
            # 计算调整比例
            ratio = min(canvas_width / width, canvas_height / height)
            new_width = int(width * ratio)
            new_height = int(height * ratio)
            # 调整图像大小
            resized_frame = cv2.resize(rgb_frame, (new_width, new_height))
            # 转换为QImage对象
            h, w, ch = resized_frame.shape
            bytes_per_line = ch * w
            q_img = QImage(resized_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            # 转换为QPixmap对象
            pixmap = QPixmap.fromImage(q_img)
            # 在QLabel上显示图像
            self.canvas.setPixmap(pixmap)

    def update_time_label(self, time_str):
        self.time_label.setText(time_str)

    def update_progress_bar(self, value):
        self.progress_bar.setValue(value)

    def set_speed(self, value):
        """设置播放速度"""
        speed = float(value)
        self.frame_delay = max(1, int(40 / speed))  # 调整延迟时间
        # 假设这里有一个显示速度的标签 self.speed_value
        # self.speed_value.setText(f"{speed:.1f}x")

    def init_video_writer(self):
        """初始化视频写入对象"""
        if self.current_frame is not None:
            height, width, _ = self.current_frame.shape
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.out = cv2.VideoWriter('temp.avi', fourcc, 20.0, (width, height))

    def start_recording(self):
        """开始录制视频"""
        self.recording = True
        if self.out is None:
            self.init_video_writer()

    def stop_recording(self):
        """停止录制视频"""
        self.recording = False
        if self.out:
            self.out.release()
            self.out = None

    def save_video(self):
        """保存视频和螺丝信息"""
        if self.playing:
            self.toggle_play()  # 暂停视频播放
        if self.recording:
            self.stop_recording()
        script_dir = os.path.dirname(os.path.abspath(__file__))
        video_name = "ros_video"
        video_path = os.path.join(script_dir, f"{video_name}_output.avi")
        yaml_path = os.path.join(script_dir, f"{video_name}_output.yaml")
        # 检查临时文件是否存在
        if not os.path.exists('temp.avi'):
            QMessageBox.warning(self, "保存失败", "未录制视频，没有临时视频文件。")
            return
        self.save_thread = threading.Thread(target=self.save_files, args=(video_path, yaml_path))
        self.save_thread.daemon = True
        self.save_thread.start()

    def save_files(self, video_path, yaml_path):
        try:
            # 保存视频
            if os.path.exists('temp.avi'):
                os.replace('temp.avi', video_path)
            # 只保存最后的螺丝信息
            if self.screw_info:
                last_screw_info = self.screw_info[-1]
                with open(yaml_path, 'w') as f:
                    yaml.dump(last_screw_info, f)
            # 显示消息框
            QMessageBox.information(self, "保存成功", f"视频已保存到 {video_path}，螺丝信息已保存到 {yaml_path}")
        except Exception as e:
            QMessageBox.warning(self, "保存失败", f"保存文件时出现错误: {str(e)}")

    def pause_stream(self):
        """暂停当前视频输入流的接收"""
        if self.playing:
            self.toggle_play()  # 暂停视频播放
        self.send_pause_command()
        print("Pause")

    def calcu_crew(self, data):
        """输入格式: [["top", 1], ["botton", 2], ...]"""
        for item in data:
            name, crew_count = item
            if name in self.names and crew_count > self.max_crew[name]:
                self.max_crew[name] = crew_count
                self.labels[name].setText(f"have max crew:{self.max_crew[name]}")

    def closeEvent(self, event):
        """处理窗口关闭事件"""
        if self.play_thread:
            self.play_thread.stop()
        if self.save_thread and self.save_thread.is_alive():
            self.save_thread.join()
        if self.cap:
            self.cap.release()
        if self.out:
            self.out.release()
        event.accept()


    # GUI.py 文件
    def set_current_result(self, result):
        print("Setting current result in GUI")
        self.current_frame = result['image']  # After rendering

        # 更新螺丝统计信息
        surface_screw_count = {surface: 0 for surface in SURFACE_CLASSES}
        for part in result['parts']:
            surface = part["on_surface"]
            if surface in surface_screw_count:
                surface_screw_count[surface] += 1
        screw_info = [[surface, count] for surface, count in surface_screw_count.items()]
        self.calcu_crew(screw_info)

        if not self.playing:
            self.display_frame(cv2.resize(result['image'], (1280, 720)))  # 新增：如果未播放，立即显示帧

if __name__ == '__main__':
    def send_pause_command():
        pass
    app = QApplication(sys.argv)  # 确保在创建GUI实例之前创建QApplication
    gui = GUI(send_pause_command)
    gui.show()
    sys.exit(app.exec_())