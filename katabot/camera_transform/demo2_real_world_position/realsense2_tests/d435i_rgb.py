import pyrealsense2 as rs
import numpy as np
import cv2

# 1. 创建pipeline
pipeline = rs.pipeline()
config = rs.config()

# 2. 配置分辨率为1280x720，帧率30fps
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# 3. 启动流
profile = pipeline.start(config)

# 4. 获取color stream的内参
color_stream = profile.get_stream(rs.stream.color)  # 获取color stream profile
intr = color_stream.as_video_stream_profile().get_intrinsics()

print("相机内参:")
print(f"宽度: {intr.width}")
print(f"高度: {intr.height}")
print(f"焦距 fx: {intr.fx}")
print(f"焦距 fy: {intr.fy}")
print(f"主点 cx: {intr.ppx}")
print(f"主点 cy: {intr.ppy}")
print(f"畸变模型: {intr.model}")
print(f"畸变参数: {intr.coeffs}")

# 5. 实时获取图像并显示
try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        cv2.imshow('RealSense Color Stream', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # 释放资源
    pipeline.stop()
    cv2.destroyAllWindows()
