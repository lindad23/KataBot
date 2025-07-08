import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import cv2
from PIL import Image, ImageTk
import threading
import time
import os
import yaml
import socket


class GUI:
    def __init__(self, send_pause_command):
        self.root = tk.Tk()
        self.root.title("Camera Transform GUI")
        self.root.geometry("1500x800")  # video_size 1200*800
        # self.root.minsize("400x300")

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
        self.thread_running = False
        self.save_thread = None

        # 发送暂停命令的回调函数
        self.send_pause_command = send_pause_command

        # Create video frame
        self.video_frame = ttk.Frame(self.root, relief="raised", width=1200, height=800)
        self.video_frame.pack(pady=20, side="left")
        self.video_frame.pack_propagate(False)  # 防止Frame根据内容调整大小

        # Create calculate frame
        self.calculate_frame = ttk.Frame(self.root, relief="raised", width=300, height=800)
        self.calculate_frame.pack(pady=20, side="right")

        # ------------ 在video_frame中添加视频播放控件 ------------

        # 创建Canvas用于显示视频
        self.canvas = tk.Canvas(self.video_frame, width=1200, height=700, bg="black")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # 创建控制面板
        self.control_frame = tk.Frame(self.video_frame, bg="#f0f0f0")
        self.control_frame.pack(fill=tk.X, pady=5)

        # 创建控制按钮
        self.play_button = tk.Button(self.control_frame, text="播放", command=self.toggle_play, state=tk.DISABLED)
        self.play_button.pack(side=tk.LEFT, padx=5)

        # 进度条和时间显示
        self.progress_frame = tk.Frame(self.video_frame, bg="#f0f0f0")
        self.progress_frame.pack(fill=tk.X, pady=5)

        self.time_label = tk.Label(self.progress_frame, text="00:00/00:00")
        self.time_label.pack(side=tk.LEFT, padx=5)

        self.progress_bar = ttk.Progressbar(self.progress_frame, orient=tk.HORIZONTAL, length=1000, mode='determinate')
        self.progress_bar.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        # ------------ 计算区域控件 ------------

        # 上部分六个框
        self.top_part = ttk.Frame(self.calculate_frame)
        self.top_part.pack(pady=20)

        self.names = ["top", "botton", "front", "back", "left", "right"]
        self.labels = {}
        self.max_crew = {name: 0 for name in self.names}

        # 定义字体大小
        font_size = 12
        # 定义边框样式
        border_width = 2
        border_color = "blue"

        for i, name in enumerate(self.names):
            frame = ttk.Frame(self.top_part, borderwidth=border_width, relief="solid")
            frame.grid(row=i // 2, column=i % 2, padx=10, pady=10, sticky="nsew")
            frame.columnconfigure(0, weight=1)
            frame.rowconfigure(0, weight=1)
            frame.rowconfigure(1, weight=1)

            name_label = tk.Label(frame, text=name, justify=tk.CENTER, font=("Arial", font_size))
            name_label.pack(fill=tk.BOTH, expand=True)

            crew_label = tk.Label(frame, text=f"have max crew:0", font=("Arial", font_size))
            crew_label.pack(fill=tk.BOTH, expand=True)

            self.labels[name] = crew_label

        # 下部分按钮
        self.bottom_part = ttk.Frame(self.calculate_frame)
        self.bottom_part.pack(pady=20)

        self.save_button = tk.Button(self.bottom_part, text="保存视频", command=self.save_video)
        self.save_button.pack(pady=10)

        self.pause_button = tk.Button(self.bottom_part, text="暂停", command=self.pause_stream)
        self.pause_button.pack(pady=10)

        # 绑定窗口关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # start
        self.root.mainloop()

    def load_video_stream(self):
        self.playing = True
        self.play_button.config(text="暂停")
        self.thread_running = True
        self.play_thread = threading.Thread(target=self.play_video)
        self.play_thread.daemon = True
        self.play_thread.start()

    def toggle_play(self):
        """切换播放/暂停状态"""
        if self.playing:
            self.playing = False
            self.play_button.config(text="播放")
            self.thread_running = False
            if self.play_thread and self.play_thread.is_alive():
                self.play_thread.join()
        else:
            self.playing = True
            self.play_button.config(text="暂停")
            self.thread_running = True
            self.play_thread = threading.Thread(target=self.play_video)
            self.play_thread.daemon = True
            self.play_thread.start()

    def play_video(self):
        """播放视频的线程函数"""
        # 用于debug的时间点和数据
        debug_times = [1, 3, 5, 8, 10]
        debug_data = [
            [["top", 1], ["botton", 2], ["right", 1]],
            [["top", 2], ["botton", 5], ["right", 6]],
            [["top", 3], ["botton", 5], ["right", 6]],
            [["top", 4], ["botton", 7], ["right", 8]],
            [["top", 5], ["botton", 9], ["right", 10]]
        ]
        debug_index = 0
        # 到这里为止

        while self.thread_running:
            start_time = time.time()

            if self.current_frame is not None:
                self.current_frame_index += 1

                # 更新进度条
                self.progress_bar["value"] = self.current_frame_index

                # 更新时间显示
                current_seconds = self.current_frame_index * (self.frame_delay / 1000)
                total_seconds = self.total_frames * (self.frame_delay / 1000)
                current_time_str = time.strftime('%M:%S', time.gmtime(current_seconds))
                total_time_str = time.strftime('%M:%S', time.gmtime(total_seconds))
                self.time_label.config(text=f"{current_time_str}/{total_time_str}")

                # 检查是否到达debug时间点
                if debug_index < len(debug_times) and current_seconds >= debug_times[debug_index]:
                    self.calcu_crew(debug_data[debug_index])
                    self.screw_info.append(debug_data[debug_index])
                    debug_index += 1

                # 显示当前帧
                self.display_frame()

                # 录制视频
                if self.recording:
                    self.out.write(self.current_frame)

            # 控制播放速度
            elapsed_time = (time.time() - start_time) * 1000  # 转换为毫秒
            sleep_time = max(0, self.frame_delay - elapsed_time)
            time.sleep(sleep_time / 1000)

    def display_frame(self):
        """显示当前帧"""
        if self.current_frame is not None:
            # 转换BGR为RGB
            rgb_frame = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)

            # 调整帧大小以适应Canvas
            height, width, _ = rgb_frame.shape
            canvas_width, canvas_height = 1200, 700

            # 计算调整比例
            ratio = min(canvas_width / width, canvas_height / height)
            new_width = int(width * ratio)
            new_height = int(height * ratio)

            # 调整图像大小
            resized_frame = cv2.resize(rgb_frame, (new_width, new_height))

            # 转换为PhotoImage对象
            image = Image.fromarray(resized_frame)
            photo = ImageTk.PhotoImage(image=image)

            # 在Canvas上显示图像
            self.canvas.delete("all")
            x_pos = (canvas_width - new_width) // 2
            y_pos = (canvas_height - new_height) // 2
            self.canvas.create_image(x_pos, y_pos, anchor=tk.NW, image=photo)

            # 保存对PhotoImage的引用，防止被垃圾回收
            self.canvas.photo = photo

    def set_speed(self, value):
        """设置播放速度"""
        speed = float(value)
        self.frame_delay = max(1, int(40 / speed))  # 调整延迟时间
        self.speed_value.config(text=f"{speed:.1f}x")

    def save_video(self):
        """保存视频和螺丝信息"""
        if self.playing:
            self.toggle_play()  # 暂停视频播放

        if self.recording:
            self.out.release()
            self.recording = False

        script_dir = os.path.dirname(os.path.abspath(__file__))
        video_name = "ros_video"
        video_path = os.path.join(script_dir, f"{video_name}_output.avi")
        yaml_path = os.path.join(script_dir, f"{video_name}_output.yaml")

        self.save_thread = threading.Thread(target=self.save_files, args=(video_path, yaml_path))
        self.save_thread.daemon = True
        self.save_thread.start()

    def save_files(self, video_path, yaml_path):
        # 保存视频
        if os.path.exists('temp.avi'):
            os.replace('temp.avi', video_path)

        # 只保存最后的螺丝信息
        if self.screw_info:
            last_screw_info = self.screw_info[-1]
            with open(yaml_path, 'w') as f:
                yaml.dump(last_screw_info, f)

        # 在主线程中显示消息框
        self.root.after(0, lambda: messagebox.showinfo("保存成功", f"视频已保存到 {video_path}，螺丝信息已保存到 {yaml_path}"))

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
                self.labels[name].config(text=f"have max crew:{self.max_crew[name]}")

    def on_closing(self):
        """处理窗口关闭事件"""
        self.thread_running = False
        self.playing = False

        if self.play_thread and self.play_thread.is_alive():
            self.play_thread.join()
        if self.save_thread and self.save_thread.is_alive():
            self.save_thread.join()

        if self.cap:
            self.cap.release()
        if self.out:
            self.out.release()

        self.root.destroy()
