import cv2
import numpy as np
# from katabot.camera_transform.demo2_real_world_position import D435iCamera as Camera
from katabot.camera_transform.demo2_real_world_position import D435Camera as Camera
from pupil_apriltags import Detector, Detection

special_tag_size = {
    11: 0.05
}

def get_extrinsics_matrix_pnp(xs_img: np.ndarray, xs_world: np.ndarray, K: np.ndarray, dist=None):
    """ Give one-to-one N (N >= 4) points in world and image coordinate systems, return world coor-sys to camera coor-sys.
    Args:
        xs_img: [shape=(N,2)] Points in image coordinate system
        xs_world: [shape=(N,3)] Points in world coordinate system
        K: [shape=(3,3)] Camera internal matrix
        dist: [shape=(4,) or None] If not None, give distortion coefficients k1, k2, p1, p2 (option: [k3, k4] after)
    Returns: (World coor-sys to Camera coor-sys)
        R: [shape=(3,3)] Rotation matrix
        T: [shape=(3,)] Translation vector

    """
    assert len(xs_img) == len(xs_world) and len(xs_img) >= 4
    if len(xs_img) > 4:  # PnP need 4 input points
        xs_img = xs_img[:4]
        xs_world = xs_world[:4]
    _, R, T = cv2.solvePnP(
        xs_world, xs_img, K, distCoeffs=dist,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    R, _ = cv2.Rodrigues(R)
    return R, T[:, 0]

class AprilTagDetection:
    corner_world_ratio_pos = [(-0.5, -0.5, 0), (0.5, -0.5, 0), (0.5, 0.5, 0), (-0.5, 0.5, 0)]

    def __init__(self, tag_family="tag36h11"):
        self.detector = Detector(families=tag_family)
    
    def detect(self, rgb_img, K=None, dist=None, tag_size=0.04, verbose=False):
        tags: list[Detection] = self.detector.detect(cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY))
        tag_infos: dict[int, list[dict]] = dict()
        for tag in tags:
            xs_img = []
            xs_world = []
            for idx, corner in enumerate(tag.corners):
                x_img = corner.copy()
                ts = tag_size if tag.tag_id not in special_tag_size else special_tag_size[tag.tag_id]
                x_world = np.array(self.corner_world_ratio_pos[idx], np.float32) * tag_size
                xs_img.append(x_img)
                xs_world.append(x_world)
            xs_img = np.array(xs_img, np.float32)
            xs_world = np.array(xs_world, np.float32)

            R, t = None, None
            if K is not None:
                R, t = get_extrinsics_matrix_pnp(xs_img, xs_world, K, dist)

            id = int(tag.tag_id)
            info = {
                'id': id,
                # [bottom-left, bottom-right, top-right, top-left]
                'corners': tag.corners,
                'R': R,  # shpae=(3,3)
                't': t,  # shape=(3,)
            }
            if id not in tag_infos:
                tag_infos[id] = [info]
            else:
                tag_infos[id].append(info)
                if verbose:
                    print(f"[WARNING]: Exist {len(tag_infos[id])}-multiple tag_{id=}")
        return tag_infos
    
    def draw_detected_corners(self, rgb_img, K=None, dist=None, tag_size=0.04, verbose=False, draw_text=True):
        img = rgb_img.copy()
        tag_infos = self.detect(img, K, dist, tag_size, verbose)

        for id in tag_infos:
            for info in tag_infos[id]:
                # [bottom-left, bottom-right, top-right, top-left]
                colors = [(255,0,0), (0,255,0), (0,0,255), (0,255,255)]
                for idx, corner in enumerate(info['corners']):
                    corner = corner.astype(np.int32)
                    cv2.circle(img, corner, 5, colors[idx], -1)
                text_org = np.mean(info['corners'], axis=0).astype(np.int32)
                if draw_text:
                    cv2.putText(img, f"Tag{id}", text_org, cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), thickness=2)
        
        return img, tag_infos

def debug_draw_corner_detect():
    april_tag_detection = AprilTagDetection()
    camera = Camera()
    while True:
        rgb_img, depth_img = camera.get_frame()
        corners_img = april_tag_detection.draw_detected_corners(rgb_img)
        cv2.imshow("debug corners", corners_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def debug_detect_and_draw_axis():
    axis_colors = [(0,0,255), (0,255,0), (255,0,0)]
    april_tag_detection = AprilTagDetection()
    camera = Camera()
    while True:
        rgb_img, depth_img = camera.get_frame()
        img, tag_infos = april_tag_detection.draw_detected_corners(
            rgb_img=rgb_img,
            K=camera.K,
            dist=np.array(camera.dist, np.float32)
        )
        for id in tag_infos:
            # print(f"{id=}, info={tag_infos[id][0]}")
            for info in tag_infos[id]:
                axis_world_pos = np.array([[0,0,0], [0.1,0,0], [0,0.1,0], [0,0,0.1]], np.float32)
                axis_img_pos = camera.world_pos2img_pos(axis_world_pos, info['R'], info['t'])
                origin_img_pos = axis_img_pos[0]
                for idx, color in enumerate(axis_colors):
                    img = cv2.line(img, origin_img_pos, axis_img_pos[idx+1], color, thickness=2)
                img = cv2.circle(img, origin_img_pos, radius=5, color=(255,255,255), thickness=-1)
        cv2.imshow("debug draw axis", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    # debug_draw_corner_detect()
    debug_detect_and_draw_axis()

