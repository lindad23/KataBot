import cv2
import numpy as np
print("Loading YOLO modules (Loading Pytorch)...", flush=True, end='')
from ultralytics import YOLO
print("OK", flush=True)
from demo5_gui.scripts.constants import (
    SURFACE_MODEL_PATH, PART_MODEL_PATH, SURFACE_CLASSES, PART_CLASSES
)

class YOLODetector:
    """ YOLODetector 类用于加载 YOLO 模型并进行表面和零件检测 """
    def __init__(
        self, surface_model_path: str = str(SURFACE_MODEL_PATH), part_model_path: str = str(PART_MODEL_PATH),
        surface_classes: list = SURFACE_CLASSES, part_classes: list = PART_CLASSES
    ):
        self.surface_model = YOLO(surface_model_path,task='segment')  # 分割任务
        self.part_model = YOLO(part_model_path, task='detect')  # 检测任务
        self.surface_classes = surface_classes
        self.part_classes = part_classes
    
    def __call__(self, img: np.ndarray, save: bool = False):
        assert img.ndim == 3, "Input image must be a 3D array (H, W, C)"
        if img.dtype == np.float32:
            img = (img * 255).astype(np.uint8)
        img_h, img_w = img.shape[:2]
        surface_result = self.surface_model.predict(source=img, conf=0.5)[0]
        part_result = self.part_model.predict(source=img, conf=0.5)[0]

        detected_surfaces = []
        detected_parts = []
    
        # 1. Process Mask Detection Results (Segmentation Task)
        if surface_result.masks is not None:
            for i in range(len(surface_result.boxes)):
                class_id = int(surface_result.boxes[i].cls)
                class_name = self.surface_model.names[class_id]
                
                if class_name in self.surface_classes:
                    raw_mask = surface_result.masks.data[i].cpu().numpy()
                    resized_mask = cv2.resize(raw_mask, (img_w, img_h), interpolation=cv2.INTER_LINEAR)
                    resized_mask = np.clip(resized_mask, 0, 1)
                    detected_surfaces.append({
                        "name": class_name,
                        "conf": float(surface_result.boxes[i].conf),
                        "mask": resized_mask
                    })
    
        # 2. Process Bounding Box Detection Results (Detection Task)
        if part_result.boxes is not None:
            for i in range(len(part_result.boxes)):
                class_id = int(part_result.boxes[i].cls)
                class_name = self.part_model.names[class_id]
                
                if class_name in self.part_classes:
                    bbox = part_result.boxes[i].xyxy[0].cpu().numpy().astype(int)
                    detected_parts.append({
                        "name": class_name,
                        "conf": float(part_result.boxes[i].conf),
                        "bbox": bbox
                    })

        # 3. Associate Parts with Surfaces
        for part in detected_parts:
            part_bbox = part['bbox']
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
        vis_img = self.visualize(img, detected_surfaces, detected_parts)
        if save:
            cv2.imwrite("prediction_result.jpg", vis_img)
            print("\n结果图片已保存为 prediction_result.jpg")

        return {
            "image": vis_img, 
            "surfaces": detected_surfaces, 
            "parts": detected_parts
        }


    def visualize(self, image: np.ndarray, surfaces: np.ndarray, parts: np.ndarray):
        """在图上绘制结果"""
        vis_img = image.copy()
        
        # 绘制表面 (半透明mask)
        for surface in surfaces:
            mask = surface['mask']  # float array with values 0 to 1
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
    
    IMAGE_TO_PREDICT = '/GUI/katabot/camera_transform/yolo_v11_detection/618.jpg' # 请修改为你的图片路径

    # --- 执行预测和关联 ---
    img = cv2.imread(IMAGE_TO_PREDICT)
    # print(img.shape)
    # cv2.resize(img, (640, 480), interpolation=cv2.INTER_LINEAR)
    yolo_detector = YOLODetector()
    yolo_detector(img, True)
