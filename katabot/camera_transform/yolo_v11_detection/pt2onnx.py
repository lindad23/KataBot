from ultralytics import YOLO

def convert_pt_to_onnx(pt_model_path: str, onnx_save_path: str, imgsz: tuple = (640, 640)):
    """
    将YOLOv8的.pt模型转换为.onnx格式
    
    Args:
        pt_model_path (str): .pt模型权重路径
        onnx_save_path (str): 保存.onnx模型的路径（含文件名）
        imgsz (tuple): 模型输入尺寸（宽, 高），默认(640, 640)
    """
    # 加载训练好的.pt模型
    model = YOLO(pt_model_path)
    
    # 导出为ONNX格式（自动处理输入输出节点）
    model.export(
        format='onnx',       # 目标格式
        imgsz=imgsz,         # 输入尺寸（需与训练时一致）
        device=0,            # 使用GPU导出（若可用，-1表示CPU）
        verbose=True         # 打印导出日志
    )
    
    print(f"ONNX模型已保存至: {onnx_save_path}")

# ---------------------- 执行转换 ----------------------
# 示例：将表面分割模型和零件检测模型转换为ONNX
convert_pt_to_onnx(
    pt_model_path='./runs/segment/my_custom_training/weights/best.pt',
    onnx_save_path='./runs/segment/my_custom_training/weights/best.onnx',
    imgsz=(384, 640)  # 需与训练时的imgsz参数一致（通常在训练命令中指定，如--img 640）
)

convert_pt_to_onnx(
    pt_model_path='./runs/detection/my_custom_training/weights/best.pt',
    onnx_save_path='./runs/detection/my_custom_training/weights/best.onnx',
    imgsz=(384, 640)
)