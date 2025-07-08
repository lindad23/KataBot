from ultralytics import YOLO
from pathlib import Path
import cv2

# 加载预训练模型（也可以使用自定义训练的模型）
model = YOLO("E:/bot/KataBot/runs/detect/train6/weights/best.pt")  # 使用YOLOv8 Nano版本，可替换为其他版本或自定义模型路径

# 输入和输出文件夹路径
input_folder = Path("E:/bot/KataBot/katabot/camera_transform/demo4_YOLO_leaves/train_pic")
output_folder = Path("E:/bot/KataBot/katabot/camera_transform/demo4_YOLO_leaves/img_vir")
output_folder.mkdir(exist_ok=True, parents=True)

# 处理所有JPG图片
for img_path in input_folder.glob("*.jpg"):
    try:
        # 进行预测
        results = model(img_path, conf=0.10)
        
        # 获取预测结果图像
        annotated_frame = results[0].plot()
        
        # 保存可视化结果
        output_path = output_folder / img_path.name
        cv2.imwrite(str(output_path), annotated_frame)
        
        print(f"已处理: {img_path.name} -> 已保存至: {output_path}")
    except Exception as e:
        print(f"处理图像 {img_path} 时出错: {e}")    