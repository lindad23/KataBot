"""
检查重复的文件名, 进行去重
"""

from pathlib import Path

if __name__ == '__main__':
    path_dir = "/home/yy/Documents/wty/项目/618/dataset/detection/train"
    paths = list(Path(path_dir).glob("**/*.jpg"))
    print(f"Total files: {len(paths)}")
    names = set([path.stem for path in paths])
    print(f"Total unique names: {len(names)}")
    for path in paths:
        if path.stem in names:
            names.remove(path.stem)
        else:
            print(f"Duplicate found: {path} remove it.")
            path.unlink()
