import cv2
import time
import numpy as np
from katabot import PATH_LOGS
from katabot.camera_transform.demo2_real_world_position import (
    AprilTagDetection, D435iCamera, D435Camera
)
PATH_VIDEO_DIR = PATH_LOGS / "demo2/"
PATH_VIDEO_DIR.mkdir(parents=True, exist_ok=True)

def detect_and_draw_axis_relative_to_base_coord(
        Camera=D435iCamera, width=1280, height=720, fps=30,
        base_frame_idx=0, use_depth=False, save=False,
        show_pos_err=[], interval_size=0.052, grid_idxs: np.ndarray=None,
        show_depth_err=False, show_pos='rgb', tag_size=0.04):
    """ Detect all ApilTags (tag36h11) in the image and display their corner points and frames.
    You can define a base frame, which other frames will use as a reference to calculate relative position.

    Args:
        base_frame_idx (int, optional): The AprilTag index for base frame, all other frames will be based on it. Defaults to 0.
        use_depth (bool, optional): Whether to use depth image to calculate relative position through 2D image points. Defaults to False.
        save (bool, optional): Whether to save video to `/logs/demo2/{timetag}.avi`. Defaults to False.
        show_pos_err (list | None, optional: list['rgb', 'depth']): Optional show of rgb, depth detection error, only used in grid tags. Defaults to None.
        interval_size (float, optional): The distance (meter) between tags in grid, xy axes have same spacing. Defaults to 0.052.
        grid_idxs (np.ndarray | None, optional): The AprilTag indexes in grid, axis-0=x-axis, axis-1=y-axis. Defaults to None.
        show_depth_err (bool, optional): Whether to show center points depth error with rgb result, only if use_depth=True. Defaults to False.
        show_pos ('str', option['rgb', 'depth']): Show position type, rgb or depth. Defaults to rgb.
        tag_size (float): The size of AprilTags. Defaults to 0.04.
    """
    if len(show_pos_err):
        assert grid_idxs is not None
    axis_colors = [(0,0,255), (0,255,0), (255,0,0)]
    april_tag_detection = AprilTagDetection()
    # camera = D435Camera(width=1280, height=720, fps=fps, only_rgb=not use_depth)
    camera = Camera(width=1280, height=720, fps=fps, only_rgb=not use_depth)
    base_frame_info = None
    if save:
        path_video = PATH_VIDEO_DIR / f"{time.strftime(r'%Y%m%d_%H%M%S')}.avi"
        print(f"Save video to {path_video}")
        save_width = width if not show_depth_err else width * 2
        writer = cv2.VideoWriter(str(path_video), cv2.VideoWriter_fourcc(*'XVID'), fps=fps, frameSize=(save_width, height))
    while True:
        rgb_img, depth_img = camera.get_frame()
        img, tag_infos = april_tag_detection.draw_detected_corners(
            rgb_img=rgb_img,
            K=camera.K,
            dist=np.array(camera.dist, np.float32),
            draw_text=False,
            tag_size=tag_size
        )
        if base_frame_idx in tag_infos:
            base_frame_info = tag_infos[base_frame_idx][0]
        else:
            base_frame_info = None
        if show_pos_err or show_depth_err:
            pos_mean_err = {'rgb': 0, 'depth': 0}
            depth_mean_err = 0
            tag_count = 0
        for id in tag_infos:
            for info in tag_infos[id]:
                if show_pos_err or show_depth_err:
                    tag_count += 1
                axis_world_pos = np.array([[0,0,0], [0.03,0,0], [0,0.03,0], [0,0,0.03]], np.float32)
                axis_img_pos = camera.world_pos2img_pos(axis_world_pos, info['R'], info['t'])
                origin_img_pos = axis_img_pos[0]
                for idx, color in enumerate(axis_colors):
                    img = cv2.line(img, origin_img_pos, axis_img_pos[idx+1], color, thickness=2)
                img = cv2.circle(img, origin_img_pos, radius=5, color=(255,255,255), thickness=-1)
                if id != base_frame_idx and base_frame_info is not None:
                    if use_depth and (show_depth_err or show_pos == 'depth' or 'depth' in show_pos_err):
                        depth_relative_pos = camera.img_pos2world_pos(origin_img_pos, depth_img, base_frame_info['R'], base_frame_info['t'])[0]
                    if show_depth_err or (show_pos == 'rgb' or 'rgb' in show_pos_err):
                        p = info['t'].reshape(1, 3)  # (1, 3)
                        rgb_relative_pos = (p - base_frame_info['t'].reshape(1, 3)) @ base_frame_info['R']
                        rgb_relative_pos = rgb_relative_pos[0]
                    relative_pos = rgb_relative_pos if show_pos == 'rgb' else depth_relative_pos

                    text_pos = origin_img_pos  # (W, H)
                    text_pos[1] += 30
                    cv2.putText(img, f"{relative_pos.round(3)}", text_pos, cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0,0,255), thickness=1)

                    if show_pos_err:
                        if np.sum(grid_idxs==id) != 1:
                            print(f"[WARNING]: Don't find {id=} in grid_idxs")
                            continue
                        base_grid_pos = np.argwhere(grid_idxs==base_frame_idx)[0]
                        now_grid_pos = np.argwhere(grid_idxs==id)[0]
                        delta_pos = now_grid_pos - base_grid_pos
                        target_relative_pos = delta_pos * interval_size
                        target_relative_pos = np.r_[target_relative_pos, 0]
                        if 'rgb' in show_pos_err:
                            error = np.abs(rgb_relative_pos - target_relative_pos)  # error x, y, z
                            pos_mean_err['rgb'] += (error - pos_mean_err['rgb']) / tag_count
                        if 'depth' in show_pos_err and use_depth:
                            error = np.abs(depth_relative_pos - target_relative_pos)  # error x, y, z
                            pos_mean_err['depth'] += (error - pos_mean_err['depth']) / tag_count
                    if show_depth_err:
                        rgb_z = info['t'][2]
                        img_idxs = origin_img_pos.astype(np.int32)
                        img_idxs[0] = np.clip(img_idxs[0], 0, depth_img.shape[1]-1)  # W
                        img_idxs[1] = np.clip(img_idxs[1], 0, depth_img.shape[0]-1)  # H
                        depth_z = depth_img[img_idxs[1], img_idxs[0]] / 1e3  # NOTE: millimeter -> meter
                        depth_mean_err += (abs(rgb_z - depth_z) - depth_mean_err) / tag_count
                        text_pos[1] += 30
                        cv2.putText(img, f"depth_z(mm)={np.round(depth_z*1e3, 3)}", text_pos, cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.8, (0,0,255), thickness=1)
                        # print(f"{img_idxs=}, {id=}, {depth_z=}")

        delta_height = 30
        left_top_txt_height = 25
        cv2.putText(img, f"pos={show_pos}", (0, left_top_txt_height), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), thickness=2)
        if 'rgb' in show_pos_err:
            left_top_txt_height += delta_height
            cv2.putText(img, f"rgb mean error={np.round(pos_mean_err['rgb'], 3)}", (0, left_top_txt_height), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), thickness=2)
        if 'depth' in show_pos_err and use_depth:
            left_top_txt_height += delta_height
            cv2.putText(img, f"depth mean error={np.round(pos_mean_err['depth'], 3)}", (0, left_top_txt_height), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), thickness=2)
        if show_depth_err:
            left_top_txt_height += delta_height
            cv2.putText(img, f"depth error={np.round(depth_mean_err, 3)}", (0, left_top_txt_height), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), thickness=2)

        if show_depth_err:
            depth_normalized = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
            depth_normalized = depth_normalized.astype(np.uint8)
            # depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            # overlay = cv2.addWeighted(rgb_img, 0.8, depth_colormap, 0.4, 0)
            # cv2.imshow("debug depth image", overlay)
            depth_normalized = np.stack([depth_normalized]*3, axis=-1)
            img = np.concatenate([img, depth_normalized], axis=1)
        cv2.imshow("debug draw axis", img)

        if save:
            writer.write(img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            if save:
                writer.release()
            break

if __name__ == '__main__':
    # grid_idxs = np.fliplr(np.arange(20, dtype=np.int32).reshape(5, 4).T)  # 0~19, 4 rows, 5 cols
    grid_idxs = np.fliplr(np.arange(6, 15, dtype=np.int32).reshape(3, 3).T)  # 6~14, 3 rows, 3 cols
    # Grid error debug
    # detect_and_draw_axis_relative_to_base_coord(Camera=D435Camera, fps=15, base_frame_idx=6, use_depth=False, save=True, show_pos_err=['rgb'], grid_idxs=grid_idxs)
    # Without Grid
    # detect_and_draw_axis_relative_to_base_coord(Camera=D435Camera, fps=15, base_frame_idx=11, use_depth=False, save=True, tag_size=0.04)
    detect_and_draw_axis_relative_to_base_coord(Camera=D435iCamera, base_frame_idx=11, use_depth=True, save=False, tag_size=0.04, show_pos_err=['rgb', 'depth'], grid_idxs=grid_idxs)

    # detect_and_draw_axis_relative_to_base_coord(use_depth=True, save=True, show_pos_err=['rgb', 'depth'], grid_idxs=grid_idxs, show_depth_err=True, show_pos='depth')
