import os
import cv2
from ultralytics import YOLO

# 预测参数
model_path = 'E:/bot/KataBot/katabot/camera_transform/demo3_YOLO/yolo_dataset/custom_yolo_obb/weights/best.pt'
img_dir = 'E:/bot/KataBot/katabot/camera_transform/demo3_YOLO/train_img'
output_dir = 'E:/bot/KataBot/katabot/camera_transform/demo3_YOLO/predict_vis'

os.makedirs(output_dir, exist_ok=True)

# 加载YOLO11-OBB模型
model = YOLO(model_path)
# model = YOLO("yolo11n-obb.pt")  # load an official model

# 颜色映射
color_map = {
    "background": (128, 128, 128),
    "top_head": (0, 255, 0),
    "top_end": (0, 0, 255)
}

# 遍历图片
img_files = [f for f in os.listdir(img_dir) if f.lower().endswith(('.jpg', '.png'))]

for img_name in img_files:
    img_path = os.path.join(img_dir, img_name)
    img = cv2.imread(img_path)
    results = model(img, conf=0.05)
    for result in results:
        names = result.names
        obb = getattr(result, "obb", None)
        if result.boxes is None:
            print(f"图片 {img_name} 未检测到目标")
            continue  # 没有检测到目标，跳过
        print(f"图片 {img_name} 检测到目标")
        classes = result.boxes.cls.cpu().numpy().astype(int)
        if obb is not None:
            polys = obb.data.cpu().numpy()
            for i, poly in enumerate(polys):
                class_id = classes[i]
                class_name = names[class_id] if class_id in names else str(class_id)
                pts = poly.reshape(4, 2).astype(int)
                color = color_map.get(class_name, (255, 255, 0))
                cv2.polylines(img, [pts], isClosed=True, color=color, thickness=2)
                # 标注类别
                cx, cy = pts.mean(axis=0).astype(int)
                cv2.putText(img, class_name, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    # 保存可视化结果
    cv2.imwrite(os.path.join(output_dir, img_name), img)

print(f"预测可视化结果已保存")