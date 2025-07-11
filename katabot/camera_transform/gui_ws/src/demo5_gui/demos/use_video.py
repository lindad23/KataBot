import cv2
import subprocess
import numpy as np
from pathlib import Path
PATH_LOGS = Path(__file__).parents[1] / "logs"
PATH_LOGS.mkdir(parents=True, exist_ok=True)

import time


np.random.seed(42)
USE_MP4V = False

### Create video writer
fps = 30
width, height = 540, 360
path_video = PATH_LOGS / (time.strftime("%Y%m%d_%H%M%S") + "_test.mp4")
if USE_MP4V:  # 文件更小, 如果创建过程中中止可能出现花屏
    writer = cv2.VideoWriter(str(path_video), cv2.VideoWriter_fourcc(*'mp4v'), fps=fps, frameSize=(width, height))
else:  # 文件稍微大, 如果创建过程中中止不会出现问题
    path_video = path_video.with_suffix('.avi')  # 使用avi格式
    writer = cv2.VideoWriter(str(path_video), cv2.VideoWriter_fourcc(*'XVID'), fps=fps, frameSize=(width, height))

duration_sec = 3
total_frames = duration_sec * fps
update_freq = 30
img = np.zeros((height, width, 3), dtype=np.uint8)
delta = np.ones((1, 1, 3), dtype=np.int32)
scale = 5

for i in range(total_frames):
    # Write image (RGB -> BGR)
    writer.write(img[..., ::-1])

    img = np.clip(img + delta * scale, 0, 255).astype(np.uint8)
    if i % update_freq == 0:
        delta = np.random.randint(-1, 2, size=(1, 1, 3), dtype=np.int32)

# Release writer
writer.release()

# Option: Use ffmpeg to compress video into mp4, for example: 202.8kB -> 7.3kB
subprocess.run(['ffmpeg', '-y', '-i', str(path_video), str(path_video.with_stem(path_video.stem+'_small').with_suffix('.mp4'))])