import pyrealsense2 as rs
import numpy as np
import cv2

# 1. 创建管线并配置流
pipeline = rs.pipeline()
config = rs.config()

# 启用 RGB 流（1920x1080@30）
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

# 启用深度流（推荐使用1280x720，若设备支持也可设为1920x1080）
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# 2. 启动管线
profile = pipeline.start(config)

# 3. 创建对齐对象：将深度图对齐到彩色图像
align = rs.align(rs.stream.color)

try:
    while True:
        # 等待对齐后的帧
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        # 获取对齐后的 RGB 和深度帧
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not depth_frame or not color_frame:
            continue

        # 转换为 NumPy 数组
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # 归一化深度图以增强可视化效果
        depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_normalized = depth_normalized.astype(np.uint8)

        # 应用伪彩色映射（热力图）
        depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

        # Resize 深度图到 RGB 大小（如有必要）
        if depth_colormap.shape[:2] != color_image.shape[:2]:
            depth_colormap = cv2.resize(depth_colormap, (color_image.shape[1], color_image.shape[0]))

        # 将深度热力图叠加到 RGB 图像上（热力图半透明）
        overlay = cv2.addWeighted(color_image, 0.6, depth_colormap, 0.4, 0)

        # 显示结果
        cv2.imshow('RGB + Depth Heatmap Overlay', overlay)

        # 按下 q 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # 停止流
    pipeline.stop()
    cv2.destroyAllWindows()
