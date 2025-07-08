from ultralytics import YOLO






# 加载预训练模型
model = YOLO('yolov8n.pt')



if __name__ == "__main__":
    # 训练模型
    results = model.train(
        data='E:/bot/KataBot/katabot/camera_transform/demo4_YOLO_leaves/data.yaml',
        epochs=10,  # 训练轮数
        imgsz= (640,480),  # 输入图像尺寸
        # project='E:/bot/KataBot/katabot/camera_transform/demo4_YOLO_leaves/train_results'  # 保存路径
        # name='yolo_leaves_train'  # 保存模型的名称
        name = 'yolo_leaves_train',  # 保存模型的名称
        augment=True,  # 启用数据增强
        mosaic=1.0,    # Mosaic增强强度
        mixup=0.5      # MixUp增强强度
    )
