import json
import os
from pathlib import Path
import cv2
import numpy as np
from sklearn.model_selection import train_test_split
from tqdm import tqdm

def convert_labelme_to_yolo(root_data_dir: Path, output_dir: Path, class_names: list, val_split=0.2):
    """
    递归扫描根数据目录，将 LabelMe 格式的标注文件转换为 YOLOv8 分割/检测格式。
    这个版本适用于图片和JSON文件存放在同一个子目录中的情况。

    Args:
        root_data_dir (Path): 存放所有数据的根目录 (例如 'train_devision_v1')。
        output_dir (Path): 输出 YOLO 格式数据集的根目录。
        class_names (list): 所有类别的名称列表。
        val_split (float): 验证集所占的比例。
    """
    # 1. 创建输出目录结构
    dir_images_train = output_dir / "images" / "train"
    dir_images_val = output_dir / "images" / "val"
    dir_labels_train = output_dir / "labels" / "train"
    dir_labels_val = output_dir / "labels" / "val"

    for p in [dir_images_train, dir_images_val, dir_labels_train, dir_labels_val]:
        p.mkdir(parents=True, exist_ok=True)

    # 2. **【修改点】** 递归地从根目录查找所有 .json 文件
    print(f"正在从 '{root_data_dir}' 递归扫描 *.json 文件...")
    json_files = sorted(list(root_data_dir.rglob("*.json")))
    if not json_files:
        print(f"错误：在 '{root_data_dir}' 及其子目录中没有找到任何 .json 文件。")
        return
    print(f"找到了 {len(json_files)} 个 JSON 文件。")

    # 3. 分割训练/验证集
    train_files, val_files = train_test_split(json_files, test_size=val_split, random_state=42)

    # 4. 创建类别映射
    class_map = {name: i for i, name in enumerate(class_names)}
    print(f"类别映射: {class_map}")

    # 5. 处理文件
    process_files(train_files, dir_images_train, dir_labels_train, class_map)
    process_files(val_files, dir_images_val, dir_labels_val, class_map)

    # 6. 创建 data.yaml 文件
    create_yaml(output_dir, str(output_dir.resolve()), class_names)
    
    print("\n转换完成！")
    print(f"YOLOv8 数据集已在 '{output_dir}' 中创建。")


def process_files(files: list, target_img_dir: Path, target_label_dir: Path, class_map: dict):
    """处理单个文件集（训练或验证）"""
    for json_file in tqdm(files, desc=f"正在处理 {target_img_dir.parent.name} 集"):
        with open(json_file, encoding='utf-8') as f:
            data = json.load(f)

        img_height = data['imageHeight']
        img_width = data['imageWidth']
        
        # **【修改点】** 在JSON文件所在的目录查找对应的图片文件
        original_img_path = find_image_file(json_file.parent, json_file.stem)
        
        if original_img_path is None:
            print(f"\n警告：在 '{json_file.parent}' 目录下找不到与 {json_file.name} 关联的图片，已跳过。")
            continue

        # 使用硬链接或复制图片到目标目录，避免占用过多空间
        target_img_path = target_img_dir / original_img_path.name
        if not target_img_path.exists():
            # os.link(original_img_path, target_img_path) # 硬链接，推荐在Linux/macOS上使用
            # 在Windows上，硬链接可能需要管理员权限，或者可以直接复制
            import shutil
            shutil.copy(original_img_path, target_img_path)

        yolo_labels = []
        for shape in data['shapes']:
            label = shape['label']
            if label not in class_map:
                print(f"\n警告: 标签 '{label}' 不在定义的 class_names 中，已跳过。文件: {json_file.name}")
                continue
            
            class_id = class_map[label]
            points = np.array(shape['points'], dtype=np.float32)

            # 根据形状类型生成YOLO字符串
            if shape['shape_type'] == 'polygon':
                # 分割任务
                normalized_points = points.flatten()
                normalized_points[0::2] /= img_width  # 归一化 x
                normalized_points[1::2] /= img_height # 归一化 y
                points_str = " ".join(map(str, normalized_points))
                yolo_labels.append(f"{class_id} {points_str}")

            elif shape['shape_type'] == 'rectangle':
                # 检测任务
                x1, y1 = points[0]
                x2, y2 = points[1]
                x_center = ((x1 + x2) / 2) / img_width
                y_center = ((y1 + y2) / 2) / img_height
                width = abs(x2 - x1) / img_width
                height = abs(y2 - y1) / img_height
                yolo_labels.append(f"{class_id} {x_center} {y_center} {width} {height}")

        # 将标签写入txt文件
        label_file_path = target_label_dir / f"{original_img_path.stem}.txt"
        with open(label_file_path, 'w', encoding='utf-8') as f:
            f.write("\n".join(yolo_labels))

def find_image_file(directory: Path, base_name: str) -> Path:
    """在指定目录中查找具有不同常见扩展名的图像。"""
    for ext in ['.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff']:
        img_path = directory / (base_name + ext)
        if img_path.exists():
            return img_path
    return None

def create_yaml(output_dir: Path, train_path: str, class_names: list):
    """创建 data.yaml 文件"""
    yaml_content = f"""
# YOLOv8 数据集配置文件
# 路径相对于 yolov8 项目根目录或者使用绝对路径
path: {train_path}  # 数据集根目录
train: images/train  # 训练集图片 (相对于 'path')
val: images/val    # 验证集图片 (相对于 'path')
# test: images/test # (可选) 测试集图片

# 类别信息
nc: {len(class_names)}  # 类别数量
names: {class_names} # 类别名称列表
"""
    with open(output_dir / "data.yaml", 'w', encoding='utf-8') as f:
        f.write(yaml_content)


if __name__ == '__main__':
    # --- 用户需要配置的参数 ---

    # 1. 定义你所有的类别名称 (顺序非常重要!)
    ALL_CLASS_NAMES = [
        # 'back', 'front', 'left', 'right', 'top', 'bottom', # 面 (分割)
        'screw1', 'screw2', 'screw3', 'screw4' # 零件 (检测)
        # ... 在这里添加你所有的其他类别
    ]

    # 2. **【修改点】** 设置你的根数据目录
    #    这个目录就是你截图中 'train_devision_v1' 所在的路径
    ROOT_DATA_DIR = Path("./train_devision_v1") # 请修改为你的根目录路径

    # 3. 设置输出YOLO数据集的目录
    # YOLO_DATASET_DIR = Path("./yolo_segment_dataset")
    YOLO_DATASET_DIR = Path("./yolo_detection_dataset")
    
    # 4. 设置验证集比例
    VALIDATION_SPLIT = 0.2

    # --- 执行转换 ---
    convert_labelme_to_yolo(
        root_data_dir=ROOT_DATA_DIR,
        output_dir=YOLO_DATASET_DIR,
        class_names=ALL_CLASS_NAMES,
        val_split=VALIDATION_SPLIT
    )