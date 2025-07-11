#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import socket
import threading
import cv2
from GUI import GUI
import os
import sys
from demo5_gui.scripts.yolo_detector import YOLODetector
from PyQt5.QtWidgets import QApplication  # 新增导入

class Ros2Interface(Node):
    def __init__(self, debug_mode=False, video_file=None):
        super().__init__('ros2_interface_node')
        self.get_logger().info('ROS2 Interface Node has been started.')
        self.bridge = CvBridge()
        self.debug_mode = debug_mode
        self.video_file = video_file
        # 模型路径和类别信息
        self.SURFACE_MODEL_PATH = 'src/demo5_gui/scripts/model/segment_best.onnx'
        self.PART_MODEL_PATH = 'src/demo5_gui/scripts/model/detection_best.onnx'
        self.yolo_detector = YOLODetector()

        # 订阅视频流
        self.video_sub = self.create_subscription(
            Image,
            'video_stream/rgb',
            self.listener_callback,
            10
        )

        # 创建Pause消息发布者
        self.pause_pub = self.create_publisher(
            String,
            'Pause_command',
            10
        )

        self.is_paused = False  # 暂停标志
        self.gui: GUI = None  # 新增：用于存储GUI实例

        # 创建Unix Domain Socket服务器
        self.server_address = '/tmp/ros2_gui_socket'
        self.socket_server = None
        self.start_socket_server()

    def set_gui(self, gui):
        self.gui = gui  # 新增：设置GUI实例

    def publish_video_frame(self):
        ret, frame = self.video_cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.video_pub.publish(img_msg)
            self.get_logger().info(f'Published video frame with width: {frame.shape[1]}, height: {frame.shape[0]}')
        else:
            self.get_logger().info('End of video file reached.')
            self.timer.cancel()

    def listener_callback(self, msg):
        # 当未收到Pause指令时处理视频流
        if self.is_paused:
            self.get_logger().info('Paused processing video frames')
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.get_logger().info(f'Received image with width: {msg.width}, height: {msg.height}')
        if self.gui:
            result = self.yolo_detector(cv_image, save=False)
            self.gui.set_current_result(result)  # 新增：将图像传递给GUI

    def start_socket_server(self):
        # 创建并启动Unix Domain Socket服务器
        try:
            # 删除可能存在的旧socket文件
            if os.path.exists(self.server_address):
                os.unlink(self.server_address)

            self.socket_server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.socket_server.bind(self.server_address)
            self.socket_server.listen(1)

            # 在后台线程中接受连接
            thread = threading.Thread(target=self._accept_connections, daemon=True)
            thread.start()
            self.get_logger().info(f'Socket server started on {self.server_address}')
        except Exception as e:
            self.get_logger().error(f'Error starting socket server: {str(e)}')

    def _accept_connections(self):
        while True:
            try:
                conn, addr = self.socket_server.accept()
                self.get_logger().info('Connection from client established')

                # 处理客户端消息
                data = conn.recv(1024)
                if data.decode() == 'Pause':
                    self.handle_pause_command()

                conn.close()
            except Exception as e:
                self.get_logger().error(f'Error handling client connection: {str(e)}')
                break

    def handle_pause_command(self):
        # 处理暂停命令
        self.is_paused = True
        # 发布Pause消息
        pause_msg = String()
        pause_msg.data = "Pause"
        self.pause_pub.publish(pause_msg)
        self.get_logger().info('Published Pause message to output_topic')
        if self.debug_mode:
            print("Published 'Pause' message as a PUBLISHER in debug mode.")

    def send_pause_command(self):
        try:
            client_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client_socket.connect(self.server_address)
            client_socket.sendall('Pause'.encode())
            client_socket.close()
        except Exception as e:
            self.get_logger().error(f'Error sending Pause command: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    ros2_interface = Ros2Interface()

    # 创建ROS节点线程
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros2_interface,), daemon=True)
    ros_thread.start()

    try:
        app = QApplication(sys.argv)  # 新增：创建QApplication实例
        # 实例化GUI对象
        gui = GUI(ros2_interface.send_pause_command)
        ros2_interface.set_gui(gui)  # 新增：设置GUI实例
        gui.show()  # 显示GUI
        sys.exit(app.exec_())  # 启动应用程序的事件循环
    # except KeyboardInterrupt:
    #     ros2_interface.get_logger().info('Node has been stopped by user.')
    # finally:
    except Exception as e:
        ros2_interface.get_logger().error(f'An error occurred: {str(e)}')
        # 清理socket资源
        if ros2_interface.socket_server:
            if os.path.exists(ros2_interface.server_address):
                os.unlink(ros2_interface.server_address)

        if hasattr(ros2_interface, 'video_cap') and ros2_interface.video_cap.isOpened():
            ros2_interface.video_cap.release()

        ros2_interface.destroy_node()
        rclpy.shutdown()
        ros_thread.join()


if __name__ == '__main__':
    main()