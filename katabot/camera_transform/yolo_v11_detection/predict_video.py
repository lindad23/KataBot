import cv2
import torch
import numpy as np
from ultralytics import YOLO

def process_frame(frame, surface_model, part_model, surface_classes, part_classes, img_h, img_w):
    """
    处理单帧图像（表面分割+零件检测+关联）
    返回：处理后的帧、检测到的表面列表、检测到的零件列表
    """
    # 加载模型（仅在首次调用时加载，这里为示例简化，实际建议全局加载）
    # 注意：原代码中模型已在主函数加载，此处保持全局加载更高效
    # surface_model = YOLO(surface_model_path, task='segment')
    # part_model = YOLO(part_model_path, task='detect')

    # 处理表面分割
    surface_results = surface_model(frame, conf=0.5)
    surface_result = surface_results[0]

    # 处理零件检测
    part_results = part_model(frame, conf=0.5)
    part_result = part_results[0]

    detected_surfaces = []
    detected_parts = []

    # 1. 处理表面分割结果
    if surface_result.masks is not None:
        for i in range(len(surface_result.boxes)):
            class_id = int(surface_result.boxes[i].cls)
            class_name = surface_model.names[class_id]
            if class_name in surface_classes:
                raw_mask = surface_result.masks.data[i].cpu().numpy()
                # 调整mask尺寸到当前帧的实际尺寸（frame可能被模型缩放）
                resized_mask = cv2.resize(raw_mask, (img_w, img_h), interpolation=cv2.INTER_LINEAR)
                resized_mask = np.clip(resized_mask, 0, 1)
                detected_surfaces.append({
                    "name": class_name,
                    "conf": float(surface_result.boxes[i].conf),
                    "mask": resized_mask
                })

    # 2. 处理零件检测结果
    if part_result.boxes is not None:
        for i in range(len(part_result.boxes)):
            class_id = int(part_result.boxes[i].cls)
            class_name = part_model.names[class_id]
            if class_name in part_classes:
                bbox = part_result.boxes[i].xyxy[0].cpu().numpy().astype(int)
                detected_parts.append({
                    "name": class_name,
                    "conf": float(part_result.boxes[i].conf),
                    "bbox": bbox
                })

    # 3. 关联零件和表面
    for part in detected_parts:
        part_bbox = part['bbox']
        part_center_x = (part_bbox[0] + part_bbox[2]) // 2
        part_center_y = (part_bbox[1] + part_bbox[3]) // 2
        found_surface = "None"

        for surface in detected_surfaces:
            surface_mask = surface['mask']
            if (0 <= part_center_y < surface_mask.shape[0] and 
                0 <= part_center_x < surface_mask.shape[1] and
                surface_mask[part_center_y, part_center_x] > 0.5):
                found_surface = surface['name']
                break
        part['on_surface'] = found_surface

    # 4. 可视化当前帧
    vis_frame = visualize(frame, detected_surfaces, detected_parts)
    return vis_frame, detected_surfaces, detected_parts

def visualize(image, surfaces, parts):
    """在图上绘制结果（与原代码一致）"""
    vis_img = image.copy()

    # 定义表面类别到固定颜色的映射
    COLOR_MAP = {
        'back': (255, 0, 0),      # 红色
        'front': (0, 255, 0),     # 绿色
        'left': (0, 0, 255),      # 蓝色
        'right': (255, 255, 0),   # 黄色
        'top': (255, 0, 255),     # 品红
        'bottom': (0, 255, 255)   # 青色
    }
    DEFAULT_COLOR = (128, 128, 128)  # 默认颜色（灰色）
    
    # 绘制表面半透明mask
    for surface in surfaces:
        mask = surface['mask']
        class_name = surface['name']
        color = COLOR_MAP.get(class_name, DEFAULT_COLOR)
        colored_mask = np.zeros_like(vis_img, dtype=np.uint8)
        colored_mask[mask > 0.5] = color
        vis_img = cv2.addWeighted(vis_img, 1, colored_mask, 0.4, 0)
    
    # 绘制零件和标签
    for part in parts:
        bbox = part['bbox']
        label = f"{part['name']} on {part['on_surface']}"
        cv2.rectangle(vis_img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
        cv2.putText(vis_img, label, (bbox[0], bbox[1] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return vis_img

def main():
    # --- 配置参数 ---
    SURFACE_MODEL_PATH = './runs/segment/my_custom_training/weights/best.pt'
    PART_MODEL_PATH = './runs/detection/my_custom_training/weights/best.pt'
    VIDEO_INPUT_PATH = './videos/2025-06-20_11-07-49_618打光检测/output.avi'
    VIDEO_OUTPUT_PATH = 'output_detected.avi'  # 输出视频路径
    SURFACE_CLASSES = ['back', 'front', 'left', 'right', 'top', 'bottom']
    PART_CLASSES = ['screw1', 'screw2', 'screw3', 'screw4']

    # --- 初始化模型 ---
    surface_model = YOLO(SURFACE_MODEL_PATH, task='segment')
    part_model = YOLO(PART_MODEL_PATH, task='detect')

    # --- 读取视频 ---
    cap = cv2.VideoCapture(VIDEO_INPUT_PATH)
    if not cap.isOpened():
        raise ValueError(f"无法打开视频文件: {VIDEO_INPUT_PATH}")

    # 获取视频基本信息
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 或根据需求改为'XVID'等

    # --- 初始化视频写入器 ---
    out = cv2.VideoWriter(VIDEO_OUTPUT_PATH, fourcc, fps, (frame_width, frame_height))

    # --- 逐帧处理 ---
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("视频处理完成或读取失败")
            break

        # 处理当前帧（注意：原代码中img_h,img_w是原图尺寸，这里直接使用视频帧尺寸）
        processed_frame, _, _ = process_frame(
            frame, 
            surface_model, 
            part_model, 
            SURFACE_CLASSES, 
            PART_CLASSES, 
            frame_height,  # 视频帧高度
            frame_width    # 视频帧宽度
        )

        # 写入输出视频
        out.write(processed_frame)

        # # 可选：实时显示处理进度（按q退出）
        # cv2.imshow('Processed Video', processed_frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

    # --- 释放资源 ---
    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()