import shutil
from pathlib import Path

if __name__ == '__main__':
    src_dir = "/home/yy/Documents/wty/项目/618/dataset/extract/618打光检测"
    # src_dir = "/home/yy/Documents/wty/项目/618/dataset/extract/618无打光"
    # src_dir = "/home/yy/Documents/wty/项目/618/dataset/extract/618有灯光远距离"
    dst_dir = "/home/yy/Documents/wty/项目/618/dataset/detection/train/top"
    print("Starting file copy from source to destination directory...")
    print("src_dir:", src_dir)
    print("dst_dir:", dst_dir)
    while True:
        name = input("Enter the name of the file to copy (or 'exit' to quit): ")
        if name.lower() == 'exit':
            print("Exiting the program.")
            break
        name = int(name)
        src = f"{src_dir}/{name:06d}.jpg"
        dst = f"{dst_dir}/{Path(src_dir).stem}_{name:06d}.jpg"
        shutil.copy(src, dst)
        print(f"Copied {src} to {dst}")
