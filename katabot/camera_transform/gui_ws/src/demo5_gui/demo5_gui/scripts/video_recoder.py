import cv2
import time
import numpy as np
from pathlib import Path
import subprocess

class VideoRecorder:
    def __init__(self, fps=30, width=540, height=360, use_mp4v=False):
        """
        初始化视频录制器
        
        Args:
            fps: 视频帧率
            width: 视频宽度
            height: 视频高度
            use_mp4v: 是否使用mp4v编码，True使用mp4v编码(文件小)，False使用XVID编码(文件大但更稳定)
        """
        self.fps = fps
        self.width = width
        self.height = height
        self.use_mp4v = use_mp4v
        self.current_frame = None
        self.recording = False
        self.frames = []
        self.start_time = None
        
        # 创建日志目录
        self.PATH_LOGS = Path(__file__).parents[1] / "logs"
        self.PATH_LOGS.mkdir(parents=True, exist_ok=True)
    
    def update_frame(self, frame):
        """更新当前帧"""
        self.current_frame = frame
        
    def startRecording(self):
        """开始录制视频"""
        if self.recording:
            print("已经在录制中...")
            return
            
        self.recording = True
        self.frames = []
        self.start_time = time.time()
        print(f"开始录制视频，帧率: {self.fps}fps")
        
    def Save(self):
        """保存录制的视频"""
        if not self.recording:
            print("没有在录制中，无法保存...")
            return
            
        self.recording = False
        elapsed_time = time.time() - self.start_time
        print(f"录制结束，持续时间: {elapsed_time:.2f}秒，总帧数: {len(self.frames)}")
        
        # 生成视频文件名
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        path_video = self.PATH_LOGS / (timestamp + "_test.mp4")
        
        # 设置编码器
        if self.use_mp4v:  # 文件更小, 如果创建过程中中止可能出现花屏
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        else:  # 文件稍微大, 如果创建过程中中止不会出现问题
            path_video = path_video.with_suffix('.avi')  # 使用avi格式
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
        
        # 创建视频写入器
        writer = cv2.VideoWriter(str(path_video), fourcc, self.fps, (self.width, self.height))
        
        # 写入所有帧
        for frame in self.frames:
            writer.write(frame)
            
        # 释放资源
        writer.release()
        print(f"视频已保存至: {path_video}")
        
        # 使用ffmpeg压缩视频(可选)
        self._compress_video(path_video)
        
    def _compress_video(self, input_path):
        """使用ffmpeg压缩视频"""
        output_path = input_path.with_stem(input_path.stem + '_small').with_suffix('.mp4')
        
        try:
            subprocess.run(['ffmpeg', '-y', '-i', str(input_path), str(output_path)], 
                          check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            print(f"视频压缩完成，压缩后文件: {output_path}")
        except subprocess.CalledProcessError as e:
            print(f"视频压缩失败: {e.stderr.decode()}")
        except FileNotFoundError:
            print("未找到ffmpeg，跳过视频压缩步骤")
            
    def record_frame(self):
        """录制当前帧(应在每一帧调用)"""
        if self.recording and self.current_frame is not None:
            # 确保帧尺寸正确
            if self.current_frame.shape[0] != self.height or self.current_frame.shape[1] != self.width:
                frame = cv2.resize(self.current_frame, (self.width, self.height))
            else:
                frame = self.current_frame
                
            # 确保是BGR格式
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                # 如果是RGB格式则转换为BGR
                if frame[0, 0, 0] > 100 and frame[0, 0, 2] < 50:  # 简单判断是否为RGB
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                # 单通道图像转为三通道
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                
            self.frames.append(frame)
            return True
        return False

# 使用示例
if __name__ == "__main__":
    # 创建录制器实例
    recorder = VideoRecorder(fps=30, width=540, height=360, use_mp4v=False)
    
    # 模拟视频帧生成
    duration_sec = 5
    total_frames = duration_sec * recorder.fps
    
    # 创建一个空白帧
    blank_frame = np.zeros((recorder.height, recorder.width, 3), dtype=np.uint8)
    
    # 开始录制
    recorder.startRecording()
    
    # 模拟视频帧捕获循环
    for i in range(total_frames):
        # 更新当前帧(这里使用随机颜色帧作为示例)
        color = (i % 255, (i*2) % 255, (i*3) % 255)
        frame = blank_frame.copy()
        cv2.rectangle(frame, (50, 50), (recorder.width-50, recorder.height-50), color, -1)
        recorder.update_frame(frame)
        
        # 录制当前帧
        recorder.record_frame()
        
        # 控制帧率(仅示例，实际应用中应根据真实帧率控制)
        time.sleep(1/recorder.fps)
    
    # 保存视频
    recorder.Save()    