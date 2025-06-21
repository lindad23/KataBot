"""
python katabot/camera_detection/frame_extractor.py \
    --path-output /home/yy/Documents/wty/项目/618/dataset/figures/detection/extract/618有灯光远距离 \
    --path-video /home/yy/Documents/wty/项目/618/dataset/videos/2025-06-20_11-16-34_618有灯光远距离/output.avi
"""

import cv2
import argparse
from tqdm import tqdm
from pathlib import Path

def parse_args():
    parser = argparse.ArgumentParser(description="Extract frames from a video file.")
    parser.add_argument("--path-video", type=str, help="Path to the video file.")
    parser.add_argument("--path-output", type=str, help="Path to save the extracted frames")
    return parser.parse_args()

class FrameExtractor:
    def __init__(self, args):
        self.name = Path(args.path_video).parent.stem
        self.cap = cv2.VideoCapture(args.path_video)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if not self.cap.isOpened():
            raise ValueError(f"Could not open video file: {args.path_video}")
        print(f"Video opened successfully: {args.path_video}")
        print(f"Video properties: Width={self.width}, Height={self.height}, FPS={self.fps}")
    
    def extract(self, interval=0.5):
        """ Extract frames from the video at a specified interval (second)."""
        interval_frame = int(interval * self.fps)
        for i in tqdm(range(self.frames)):
            flag, frame = self.cap.read()
            if not flag:
                print("End of video or failed to read frame.")
                break
            if i % interval_frame != 0:
                continue
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1)
            cv2.imwrite(f"{args.path_output}/{i:06d}.jpg", frame)
            # cv2.imwrite(f"{args.path_output}/{self.name}_frame_{i:06d}.jpg", frame)
            if key == ord('q'):
                print("Press 'q' to quit.")
                break
        self.cap.release()
        print("Video file read successfully.")

if __name__ == '__main__':
    args = parse_args()
    frame_extractor = FrameExtractor(args)
    frame_extractor.extract()
