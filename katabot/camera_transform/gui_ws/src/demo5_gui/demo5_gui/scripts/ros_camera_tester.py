#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

class RosCamera(Node):
    def __init__(self):
        super().__init__('ros_camera_test_node')
        # self.get_logger().info('ROS Camera Node has been started.')
        self.bridge = CvBridge()

        # 相机配置
        self.width = 1280
        self.height = 720
        self.fps = 30
        # self.pipeline = rs.pipeline()
        # self.config = rs.config()
        # self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        # self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, self.fps)
        # print("Building camera frame pipeline...", flush=True, end='')
        # self.profile = self.pipeline.start(self.config)
        # print("OK")
        # self.align = rs.align(rs.stream.color)


        self.cap = cv2.VideoCapture('/GUI/katabot/camera_transform/yolo_v11_detection/videos/2025-06-20_11-07-49_618打光检测/output.avi')

        # 创建发布者
        self.rgb_pub = self.create_publisher(Image, 'video_stream/rgb', 10)
        self.depth_pub = self.create_publisher(Image, 'video_stream/depth', 10)

        # 创建定时器，定时发布图像
        self.timer = self.create_timer(1 / self.fps, self.publish_frames)

    def publish_frames(self):
        # frames = self.pipeline.wait_for_frames()
        # aligned_frames = self.align.process(frames)
        print("Publishing frames...", flush=True, end='')
        # color_frame = aligned_frames.get_color_frame()
        # depth_frame = aligned_frames.get_depth_frame()
        if self.cap.isOpened():
            ret, color_frame = self.cap.read()

        # if not color_frame or not depth_frame:
        #     return

        rgb_img = np.asanyarray(color_frame)
        # depth_img = np.asanyarray(depth_frame.get_data())

        # 转换为 ROS2 消息并发布
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_img, "bgr8")
        # depth_msg = self.bridge.cv2_to_imgmsg(depth_img, "16UC1")

        self.rgb_pub.publish(rgb_msg)
        # self.depth_pub.publish(depth_msg)
        # self.get_logger().info('Published RGB and depth frames.')


def main(args=None):
    rclpy.init(args=args)
    ros_camera = RosCamera()

    try:
        rclpy.spin(ros_camera)
    except KeyboardInterrupt:
        ros_camera.get_logger().info('Node has been stopped by user.')
    finally:
        # 停止相机管道
        # ros_camera.pipeline.stop()
        ros_camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()