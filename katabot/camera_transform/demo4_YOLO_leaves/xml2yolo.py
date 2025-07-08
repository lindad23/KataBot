import os
import xml.etree.ElementTree as ET
import shutil

def convert_xml_to_yolo(xml_file, img_width, img_height, class_mapping):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    yolo_lines = []
    for obj in root.findall('object'):
        class_name = obj.find('name').text
        class_id = class_mapping[class_name]

        bbox = obj.find('bndbox')
        xmin = int(bbox.find('xmin').text)
        ymin = int(bbox.find('ymin').text)
        xmax = int(bbox.find('xmax').text)
        ymax = int(bbox.find('ymax').text)

        # 计算中心点和宽高
        x_center = (xmin + xmax) / (2 * img_width)
        y_center = (ymin + ymax) / (2 * img_height)
        width = (xmax - xmin) / img_width
        height = (ymax - ymin) / img_height

        yolo_lines.append(f"{class_id} {x_center} {y_center} {width} {height}")

    return yolo_lines

def convert_folder(xml_folder, output_folder, class_mapping):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for xml_file in os.listdir(xml_folder):
        if xml_file.endswith('.xml'):
            xml_path = os.path.join(xml_folder, xml_file)
            tree = ET.parse(xml_path)
            root = tree.getroot()

            size = root.find('size')
            img_width = int(size.find('width').text)
            img_height = int(size.find('height').text)

            yolo_lines = convert_xml_to_yolo(xml_path, img_width, img_height, class_mapping)

            txt_file = os.path.splitext(xml_file)[0] + '.txt'
            txt_path = os.path.join(output_folder, txt_file)
            with open(txt_path, 'w') as f:
                f.write('\n'.join(yolo_lines))

            # 复制对应的.jpg文件到输出文件夹
            img_name = os.path.splitext(xml_file)[0] + '.jpg'
            img_path = os.path.join(xml_folder, img_name)
            if os.path.exists(img_path):
                shutil.copy2(img_path, output_folder)
            else:
                print(f"Warning: {img_name} not found in {xml_folder}")

# 定义类别映射
class_mapping = {'zayan': 0, 'qingyan': 1}

# 输入 XML 文件夹和输出 YOLO 标注文件夹
xml_folder = 'E:/bot/KataBot/katabot/camera_transform/demo4_YOLO_leaves/train_pic'
output_folder = 'E:/bot/KataBot/katabot/camera_transform/demo4_YOLO_leaves/train_labels'

convert_folder(xml_folder, output_folder, class_mapping)