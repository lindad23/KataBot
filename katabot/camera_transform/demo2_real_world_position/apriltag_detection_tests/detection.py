import cv2
import numpy as np
from pupil_apriltags import Detector, Detection
from pathlib import Path
PATH_PARENT = Path(__file__).parent
path_demo_img = PATH_PARENT / "apriltag_demo2.jpg"

# 加载图像（灰度）
img = cv2.imread(str(path_demo_img))
size = (np.array(img.shape[:2]) * 1500 / img.shape[1]).astype(np.int32)[::-1]
img = cv2.resize(img, size, interpolation=cv2.INTER_LINEAR)

# 初始化 AprilTag 检测器
at_detector = Detector(families="tag36h11")

# 检测
tags: list[Detection] = at_detector.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))

# 画出角点
for tag in tags:
    print("Tag ID:", tag.tag_id)
    # 角点顺序为：[bottom-left, bottom-right, top-right, top-left]
    colors = [(255,0,0), (0,255,0), (0,0,255), (0,255,255)]
    print(tag)
    for idx, corner in enumerate(tag.corners):
        corner = tuple(int(i) for i in corner)
        cv2.circle(img, corner, 5, colors[idx], -1)
        print(f"Corner {idx}: {corner}")
    text_org = np.mean(tag.corners, axis=0).astype(np.int32)
    cv2.putText(img, f"Tag{tag.tag_id}", text_org, cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), thickness=2)

# 显示结果
cv2.imshow("Tag Corners", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
