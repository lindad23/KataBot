import cv2
import torch
import numpy as np
from ultralytics import YOLO

def predict_and_associate(surface_model_path: str, part_model_path: str, 
                         image_path: str, surface_classes: list, part_classes: list):
    """
    使用训练好的YOLOv8模型进行预测，并关联表面和零件。

     Args:
        surface_model_path (str): 表面检测模型权重路径 (.pt文件)。
        part_model_path (str): 零件检测模型权重路径 (.pt文件)。
        image_path (str): 要预测的图片路径。
        surface_classes (list): 表面的类别名称。
        part_classes (list): 零件的类别名称。
    """
    # 加载两个模型
    surface_model = YOLO(surface_model_path,task='segment')  # 分割任务
    part_model = YOLO(part_model_path, task='detect')  # 检测任务
    
    # 加载图像并获取原图尺寸
    img = cv2.imread(image_path)
    print(f"这张图片的尺寸是：")
    print(type(img), img.shape)  # 调试输出
    img_h, img_w = img.shape[:2]
    print(img.max(), img.min())

    print(f"开始检测平面")
    # 进行表面预测（分割）
    surface_results = surface_model.predict(source=img, conf=0.5)
    surface_result = surface_results[0]  # 获取第一张图的结果
    print(f"检测平面完成")
    print(f"开始检测零件")
    # 进行零件预测（检测）
    part_results = part_model.predict(source=img, conf=0.5)
    part_result = part_results[0]  # 获取第一张图的结果
    print(f"零件检测完成")
    # 存储检测到的表面和零件
    detected_surfaces = []
    detected_parts = []
    
    # 1. 处理表面检测结果 (分割任务)
    if surface_result.masks is not None:
        for i in range(len(surface_result.boxes)):
            class_id = int(surface_result.boxes[i].cls)
            class_name = surface_model.names[class_id]
            
            if class_name in surface_classes:
                raw_mask = surface_result.masks.data[i].cpu().numpy()
                resized_mask = cv2.resize(raw_mask, (img_w, img_h), interpolation=cv2.INTER_LINEAR)
                resized_mask = np.clip(resized_mask, 0, 1)
                detected_surfaces.append({
                    "name": class_name,
                    "conf": float(surface_result.boxes[i].conf),
                    "mask": resized_mask  # 获取mask数据
                })
    
    # 2. 处理零件检测结果 (检测任务)
    if part_result.boxes is not None:
        for i in range(len(part_result.boxes)):
            class_id = int(part_result.boxes[i].cls)
            class_name = part_model.names[class_id]
            
            if class_name in part_classes:
                # 获取边界框坐标 (xyxy格式)
                bbox = part_result.boxes[i].xyxy[0].cpu().numpy().astype(int)
                detected_parts.append({
                    "name": class_name,
                    "conf": float(part_result.boxes[i].conf),
                    "bbox": bbox
                })

    # 3. 关联零件和表面
    # 逻辑：检查每个零件的中心点是否落在某个表面的mask内
    for part in detected_parts:
        part_bbox = part['bbox']
        # 计算零件中心点
        part_center_x = (part_bbox[0] + part_bbox[2]) // 2
        part_center_y = (part_bbox[1] + part_bbox[3]) // 2
        
        found_surface = "None"
        for surface in detected_surfaces:
            surface_mask = surface['mask']
            # 检查中心点是否在mask内，使用阈值0.5
            if (part_center_y < surface_mask.shape[0] and 
                part_center_x < surface_mask.shape[1] and
                surface_mask[part_center_y, part_center_x] > 0.5):
                
                found_surface = surface['name']
                break  # 找到后即可停止
        
        part['on_surface'] = found_surface
        print(f"零件: {part['name']} (置信度: {part['conf']:.2f}) -> 位于表面: {part['on_surface']}")

    # 4. 可视化结果
    vis_img = visualize(img, detected_surfaces, detected_parts)
    cv2.imwrite("prediction_result.jpg", vis_img)
    print("\n结果图片已保存为 prediction_result.jpg")

    return {
        "image": vis_img, 
        "surfaces": detected_surfaces, 
        "parts": detected_parts
    }


def visualize(image, surfaces, parts):
    """在图上绘制结果"""
    vis_img = image.copy()
    
    # 绘制表面 (半透明mask)
    for surface in surfaces:
        mask = surface['mask']  # Float array with values 0 to 1
        # 创建一个彩色的mask
        color = np.random.randint(0, 255, (1, 3), dtype=np.uint8)
        colored_mask = np.zeros_like(vis_img, dtype=np.uint8)
        colored_mask[mask > 0.5] = color
        # 将彩色mask与原图叠加
        vis_img = cv2.addWeighted(vis_img, 1, colored_mask, 0.4, 0)
    
    # 绘制零件和标签
    for part in parts:
        bbox = part['bbox']
        label = f"{part['name']} on {part['on_surface']}"
        cv2.rectangle(vis_img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
        cv2.putText(vis_img, label, (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    return vis_img


if __name__ == '__main__':
    # --- 用户需要配置的参数 ---
    
    # 1. 表面和零件的模型权重路径
    SURFACE_MODEL_PATH = '/GUI/katabot/camera_transform/yolo_v11_detection/runs/segment/my_custom_training/weights/best.onnx'  # 表面分割模型
    PART_MODEL_PATH = '/GUI/katabot/camera_transform/yolo_v11_detection/runs/detection/my_custom_training/weights/best.onnx'  # 零件检测模型

    # 2. 要预测的图片
    IMAGE_TO_PREDICT = '/GUI/katabot/camera_transform/yolo_v11_detection/618.jpg' # 请修改为你的图片路径

    # 3. 再次定义表面和零件的类别，用于逻辑判断
    #    **必须**和训练时的 class_names 里的名字一致
    SURFACE_CLASSES = ['back', 'front', 'left', 'right', 'top', 'bottom']
    PART_CLASSES = ['screw1', 'screw2', 'screw3', 'screw4'] # 添加你所有的零件类别

    # --- 执行预测和关联 ---
    predict_and_associate(SURFACE_MODEL_PATH, PART_MODEL_PATH, IMAGE_TO_PREDICT, SURFACE_CLASSES, PART_CLASSES)