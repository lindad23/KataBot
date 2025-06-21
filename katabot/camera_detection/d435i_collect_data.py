import cv2
import math
import time
from katabot.utils.camera.d435i_camera import D435iCamera
from katabot import PATH_LOGS

path_collect_dir = PATH_LOGS / "collect_data" / time.strftime("%Y-%m-%d_%H-%M-%S")
path_collect_dir.mkdir(exist_ok=True, parents=True)

class D435iDataCollector:
    def __init__(self, camera: D435iCamera):
        self.camera = camera

    def collect_data(self, total_samples: int = math.inf, save_interval: int = -1):
        i = 0
        while i < total_samples:
            i += 1
            rgb_img, depth_img = self.camera.get_frame()
            cv2.imshow("RGB Image", rgb_img)
            cv2.imshow("Depth Image", depth_img)
            key = cv2.waitKey(1)
            if key == ord('q'):
                print("Press 'q' quiting.")
                break
            if key == ord('s') or (save_interval > 0 and i % save_interval == 0):
                filename = str(path_collect_dir / f"{i:06d}.png")
                cv2.imwrite(filename, rgb_img)
                print(f"Saved {filename}")
        cv2.destroyAllWindows()
        
    def collect_video(self, total_samples: int = math.inf):
        i = 0
        width, height, fps = self.camera.width, self.camera.height, self.camera.fps
        path_video = str(path_collect_dir / "output.avi")
        writer = cv2.VideoWriter(str(path_video), cv2.VideoWriter_fourcc(*'XVID'), fps=fps, frameSize=(width, height))
        while i < total_samples:
            i += 1
            rgb_img, depth_img = self.camera.get_frame()
            cv2.imshow("RGB Image", rgb_img)
            key = cv2.waitKey(1)
            writer.write(rgb_img)
            if key == ord('q'):
                print("Press 'q' quiting.")
                break
        writer.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    fps = 30
    # camera = D435iCamera(width=1280, height=720, fps=fps, enable_rgb=True, enable_depth=True)
    camera = D435iCamera(width=1920, height=1080, fps=fps, enable_rgb=True, enable_depth=True)
    collector = D435iDataCollector(camera)
    # collector.collect_data(save_interval=fps//2)
    collector.collect_video()
    camera.close()
