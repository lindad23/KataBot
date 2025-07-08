import cv2
import numpy as np
from ultralytics import YOLO
from katabot.camera_transform.demo2_real_world_position import D435iCamera

# 初始化D435i相机
camera = D435iCamera(width=1280, height=720, fps=30)

# 加载YOLOv8分割模型
# model = YOLO('E:/bot/KataBot/katabot/camera_transform/demo3_YOLO/yolo_dataset/custom_yolo_seg/weights/best.pt')
model = YOLO('yolo11n-obb.pt')  # 使用YOLO11-OBB模型

def get_polygon_center(polygon):
    pts = np.array(polygon).reshape(-1, 2)
    return np.mean(pts, axis=0)

def get_depth_at_point(depth_img, point):
    x, y = int(point[0]), int(point[1])
    h, w = depth_img.shape
    x = np.clip(x, 0, w-1)
    y = np.clip(y, 0, h-1)
    return depth_img[y, x] / 1000.0  # mm->m

while True:
    rgb_img, depth_img = camera.get_frame()
    results = model(rgb_img, conf=0.15)
    top_head_poly = None
    top_end_poly = None

    for result in results:
        names = result.names
        masks = result.masks
        # 检查 result.boxes 是否为 None
        if result.boxes is None:
            continue
        classes = result.boxes.cls.cpu().numpy().astype(int)
        if masks is None:
            continue
        for i, mask in enumerate(masks.data.cpu().numpy()):
            class_id = classes[i]
            class_name = names[class_id] if class_id in names else str(class_id)
            # 只保留第一个top_head和top_end
            if class_name == "top_head" and top_head_poly is not None:
                continue
            if class_name == "top_end" and top_end_poly is not None:
                continue
            # 提取多边形轮廓
            mask_bin = (mask > 0.5).astype(np.uint8)
            contours, _ = cv2.findContours(mask_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            # 取最大轮廓
            contour = max(contours, key=cv2.contourArea)
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            polygon = approx.reshape(-1, 2)
            if class_name == "top_head":
                top_head_poly = polygon
            elif class_name == "top_end":
                top_end_poly = polygon
            # 绘制分割区域
            cv2.drawContours(rgb_img, [polygon], -1, (0, 255, 0), 2)
            # 标注类别
            center = get_polygon_center(polygon)
            cv2.putText(rgb_img, class_name, tuple(center.astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # 坐标系构建
    if top_head_poly is not None and top_end_poly is not None and len(top_head_poly) >= 4 and len(top_end_poly) >= 2:
        # 1. 原点：两个目标面边界的中点
        min_dist = float('inf')
        edge1, edge2 = None, None
        for p1 in top_head_poly:
            for p2 in top_end_poly:
                dist = np.linalg.norm(p1 - p2)
                if dist < min_dist:
                    min_dist = dist
                    edge1, edge2 = p1, p2
        dists = [np.linalg.norm(pt - edge1) for pt in top_head_poly]
        far_idx = np.argsort(dists)[-2:]
        far_pts = top_head_poly[far_idx]
        origin = (edge1 + edge2) / 2
        far_center = np.mean(far_pts, axis=0)
        # 三维向量
        origin3d = np.array([origin[0], origin[1], get_depth_at_point(depth_img, origin)])
        far_center3d = np.array([far_center[0], far_center[1], get_depth_at_point(depth_img, far_center)])
        edge1_3d = np.array([edge1[0], edge1[1], get_depth_at_point(depth_img, edge1)])
        edge2_3d = np.array([edge2[0], edge2[1], get_depth_at_point(depth_img, edge2)])
        # X轴
        x_dir = far_center3d - origin3d
        if np.linalg.norm(x_dir) == 0:
            x_dir = np.array([1, 0, 0])
        else:
            x_dir = x_dir / np.linalg.norm(x_dir)
        # Y轴
        y_dir = edge2_3d - edge1_3d
        if np.linalg.norm(y_dir) == 0:
            y_dir = np.array([0, 1, 0])
        else:
            y_dir = y_dir / np.linalg.norm(y_dir)
        # Z轴（右手定则）
        z_dir = np.cross(x_dir, y_dir)
        if np.linalg.norm(z_dir) == 0:
            z_dir = np.array([0, 0, 1])
        else:
            z_dir = z_dir / np.linalg.norm(z_dir)
        # 绘制坐标系（只取前两个分量用于像素平面显示）
        scale = 80
        o = tuple(origin.astype(int))
        x_end = tuple((origin + x_dir[:2] * scale).astype(int))
        y_end = tuple((origin + y_dir[:2] * scale).astype(int))
        z_end = tuple((origin + z_dir[:2] * scale).astype(int))
        cv2.arrowedLine(rgb_img, o, x_end, (0, 0, 255), 3, tipLength=0.2)
        cv2.arrowedLine(rgb_img, o, y_end, (0, 255, 0), 3, tipLength=0.2)
        cv2.arrowedLine(rgb_img, o, z_end, (255, 0, 0), 3, tipLength=0.2)
        cv2.putText(rgb_img, "O", o, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(rgb_img, "X", x_end, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(rgb_img, "Y", y_end, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(rgb_img, "Z", z_end, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        # 输出相机坐标系与世界坐标系原点的距离
        cam_origin = np.array([rgb_img.shape[1] // 2, rgb_img.shape[0] // 2])
        cam_origin_depth = get_depth_at_point(depth_img, cam_origin)
        dist = np.linalg.norm(origin3d - np.array([cam_origin[0], cam_origin[1], cam_origin_depth]))
        cv2.putText(rgb_img, f"Cam-World Origin Dist: {dist:.2f} px/m", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    cv2.imshow("Object Seg & World Frame", rgb_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()