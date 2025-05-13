import cv2
import time
import numpy as np
from katabot import PATH_LOGS
from katabot.camera_transform.demo2_real_world_position import (
    AprilTagDetection, D435iCamera, D435Camera
)
PATH_VIDEO_DIR = PATH_LOGS / "demo2/"
PATH_VIDEO_DIR.mkdir(parents=True, exist_ok=True)

def detect_and_draw_axis_relative_to_base_coord(base_frame_idx=11, use_depth=False, save=False):
    axis_colors = [(0,0,255), (0,255,0), (255,0,0)]
    april_tag_detection = AprilTagDetection()
    width, height, fps = 1280, 720, 15
    camera = D435Camera(width=1280, height=720, fps=fps, only_rgb=True)
    base_frame_info = None
    if save:
        path_video = PATH_VIDEO_DIR / f"{time.strftime(r'%Y%m%d_%H%M%S')}.avi"
        print(f"Save video to {path_video}")
        writer = cv2.VideoWriter(str(path_video), cv2.VideoWriter_fourcc(*'XVID'), fps=fps, frameSize=(width, height))
    while True:
        rgb_img, depth_img = camera.get_frame()
        img, tag_infos = april_tag_detection.draw_detected_corners(
            rgb_img=rgb_img,
            K=camera.K,
            dist=np.array(camera.dist, np.float32),
            draw_text=False
        )
        if base_frame_idx in tag_infos and base_frame_info is None:
            base_frame_info = tag_infos[base_frame_idx][0]
        for id in tag_infos:
            for info in tag_infos[id]:
                axis_world_pos = np.array([[0,0,0], [0.05,0,0], [0,0.05,0], [0,0,0.05]], np.float32)
                axis_img_pos = camera.world_pos2img_pos(axis_world_pos, info['R'], info['t'])
                origin_img_pos = axis_img_pos[0]
                for idx, color in enumerate(axis_colors):
                    img = cv2.line(img, origin_img_pos, axis_img_pos[idx+1], color, thickness=2)
                img = cv2.circle(img, origin_img_pos, radius=5, color=(255,255,255), thickness=-1)
                if id != base_frame_idx and base_frame_info is not None:
                    if use_depth:
                        relative_pos = camera.img_pos2world_pos(origin_img_pos, depth_img, base_frame_info['R'], base_frame_info['t'])[0]
                    else:
                        p = info['t'].reshape(1, 3)  # (1, 3)
                        relative_pos = (p - base_frame_info['t'].reshape(1, 3)) @ base_frame_info['R']
                        relative_pos = relative_pos[0]
                    text_pos = origin_img_pos
                    text_pos[1] += 30
                    cv2.putText(img, f"{relative_pos.round(3)}", text_pos, cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0,0,255), thickness=1)
                    # print(relative_pos)

        if save:
            writer.write(img)

        cv2.imshow("debug draw axis", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            writer.release()
            break

if __name__ == '__main__':
    detect_and_draw_axis_relative_to_base_coord(use_depth=False, save=True)
