import torch
from ultralytics import YOLO

def train_yolo():
    # --- 用户需要配置的参数 ---

    # 1. 指定数据集配置文件 data.yaml 的路径
    #    这个文件是上一步生成的
    DATASET_CONFIG_PATH = './yolo_segment_dataset/data.yaml'

    # 2. 选择一个预训练模型
    #    yolov8n-seg.pt 是最小最快的模型，适合入门
    #    yolov8m-seg.pt 是中等大小，精度更高
    #    yolov8l-seg.pt 是较大的模型，精度最好但速度慢
    PRETRAINED_MODEL = 'yolov8n-seg.pt'

    # 3. 设置训练超参数
    EPOCHS = 100        # 训练轮次
    IMAGE_SIZE = 640   # 输入图片大小
    BATCH_SIZE = 24     # 每批次的图片数量，根据你的GPU显存调整
    
    # --- 开始训练 ---

    # 检查是否有可用的GPU
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"使用的设备: {device}")

    # 加载预训练模型
    model = YOLO(PRETRAINED_MODEL)

    # 训练模型
    print("开始训练...")
    model.train(
        data=DATASET_CONFIG_PATH,
        epochs=EPOCHS,
        imgsz=IMAGE_SIZE,
        batch=BATCH_SIZE,
        device=0, # 如果有多个GPU，可以指定使用哪一个，'0'代表第一个
        name='my_custom_training', # 训练结果将保存在 runs/segment/my_custom_training 目录下
        project='runs/segment'      # 指定项目名称
    )

    print("\n训练完成！")
    print(f"模型和结果保存在: runs/segment/my_custom_training")
    print(f"最好的模型权重是: runs/segment/my_custom_training/weights/best.pt")

if __name__ == '__main__':
    train_yolo()