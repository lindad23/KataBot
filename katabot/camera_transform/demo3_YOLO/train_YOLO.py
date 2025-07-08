import os
import shutil
import json
from glob import glob
from PIL import Image

# 1. 创建YOLO格式数据集目录结构
def create_yolo_dirs(base_dir):
    for split in ['images', 'labels']:
        for sub in ['train']:
            os.makedirs(os.path.join(base_dir, split, sub), exist_ok=True)

# 2. 保留background、top_end和top_head类别
TARGET_CLASSES = ['background', 'top_end', 'top_head']

def convert_labelme_to_yolo_seg(labelme_json, img_w, img_h, class_map):
    yolo_labels = []
    with open(labelme_json, 'r', encoding='utf-8') as f:
        data = json.load(f)
    for shape in data['shapes']:
        label = shape['label']
        if label not in TARGET_CLASSES:
            continue
        if label not in class_map:
            class_map[label] = len(class_map)
        class_id = class_map[label]
        points = shape['points']
        seg = []
        for x, y in points:
            seg.append(x / img_w)
            seg.append(y / img_h)
        seg_str = " ".join([str(class_id)] + [f"{v:.6f}" for v in seg])
        yolo_labels.append(seg_str)
    return yolo_labels

def prepare_yolo_dataset(labelme_dir, yolo_dir):
    create_yolo_dirs(yolo_dir)
    class_map = {}
    img_files = glob(os.path.join(labelme_dir, "*.jpg")) + glob(os.path.join(labelme_dir, "*.png"))
    for img_path in img_files:
        base = os.path.splitext(os.path.basename(img_path))[0]
        json_path = os.path.join(labelme_dir, base + ".json")
        if not os.path.exists(json_path):
            continue
        # 复制图片
        dst_img = os.path.join(yolo_dir, "images", "train", os.path.basename(img_path))
        shutil.copy(img_path, dst_img)
        # 读取图片尺寸
        with Image.open(img_path) as im:
            w, h = im.size
        # 转换标签
        yolo_labels = convert_labelme_to_yolo_seg(json_path, w, h, class_map)
        dst_label = os.path.join(yolo_dir, "labels", "train", base + ".txt")
        with open(dst_label, "w") as f:
            f.write("\n".join(yolo_labels))
    # 保存类别文件
    with open(os.path.join(yolo_dir, "classes.txt"), "w") as f:
        for k, v in sorted(class_map.items(), key=lambda x: x[1]):
            f.write(f"{k}\n")
    return class_map

# 3. 生成YOLO训练配置文件
def write_data_yaml(yolo_dir, class_map):
    with open(os.path.join(yolo_dir, "data.yaml"), "w") as f:
        f.write(f"path: {yolo_dir}\n")
        f.write("train: images/train\n")
        f.write("val: images/train\n")
        f.write(f"nc: {len(class_map)}\n")
        f.write(f"names: {list(class_map.keys())}\n")
        f.write("task: segment\n")  # 关键

if __name__ == "__main__":
    labelme_dir = os.path.join(os.path.dirname(__file__), "train_img")  # 你的数据目录
    yolo_dir = os.path.join(os.path.dirname(__file__), "yolo_dataset")
    class_map = prepare_yolo_dataset(labelme_dir, yolo_dir)
    write_data_yaml(yolo_dir, class_map)
    print("YOLO数据集已准备好，开始训练...")

    # 4. 使用ultralytics的yolo11n-obb模型训练
    from ultralytics import YOLO
    model = YOLO('yolo11n-obb.pt')
    results = model.train(
        data=os.path.join(yolo_dir, "data.yaml"),
        epochs=100,
        imgsz=640,
        project=yolo_dir,
        name="custom_yolo_obb"
    )
    print("训练完成，模型保存在:", results.save_dir)