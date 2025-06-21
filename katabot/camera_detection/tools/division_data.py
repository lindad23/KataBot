from pathlib import Path
import shutil

if __name__ == '__main__':
    src_dir = "/home/yy/Documents/wty/项目/618/dataset/detection/train"
    dst_dir = "/home/yy/Documents/wty/项目/618/dataset/detection/train_devision"
    num_division = 4
    paths = list(sorted(Path(src_dir).glob("**/*.jpg")))
    print(f"Total files: {len(paths)}")
    size = (len(paths) + num_division - 1) // num_division
    for i in range(num_division):
        start = i * size
        end = min((i + 1) * size, len(paths))
        if start >= len(paths):
            break
        print(f"Processing division {i + 1}: {start} to {end}")
        for path in paths[start:end]:
            division_dir = Path(dst_dir) / f"division_{i + 1}" / path.parent.name
            division_dir.mkdir(exist_ok=True, parents=True)
            dst_path = division_dir / path.name
            shutil.copy(path, dst_path)
            print(f"Copy {path} to {dst_path}")
