import cv2
import numpy as np
import pyrealsense2 as rs

class D435iCamera:
    dist_coefs = {  # D435i K38179-110 233522077098
        '1920x1080': [0.09555518, -0.23270808, -0.00439063, 0.00780472],
        '1280x720': [0.1346338, -0.22741977, -0.00169876, 0.00183166],
    }

    def __init__(self, width=1920, height=1080, fps=30, **kwargs):
        self.fps = fps
        self.width, self.height = width, height
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, fps)
        print("Building D435i camera frame pipline...", flush=True, end='')
        self.profile = self.pipeline.start(self.config)
        print("OK")
        self.align = rs.align(rs.stream.color)
        color_stream = self.profile.get_stream(rs.stream.color)
        self.intr = color_stream.as_video_stream_profile().get_intrinsics()
        self.K_coefs = [self.intr.fx, self.intr.fy, self.intr.ppx, self.intr.ppy]
        self.K = np.array([
            [self.K_coefs[0], 0, self.K_coefs[2]],
            [0, self.K_coefs[1], self.K_coefs[3]],
            [0, 0, 1],
        ], np.float32)
        self.dist = self.dist_coefs[f"{width}x{height}"]
        print("Waiting for camera start...", end='', flush=True)
        for i in range(fps*3):
            if i % fps == 0:
                print(f"{3-i//fps}s...", end='', flush=True)
            self.get_frame()
        print("OK")
    
    def close(self):
        self.pipeline.stop()
        print("D435i camera closed.", flush=True)
    
    def get_frame(self) -> tuple[np.ndarray, np.ndarray]:
        while True:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame: continue

            rgb_img = np.asanyarray(color_frame.get_data())
            depth_img = np.asanyarray(depth_frame.get_data())
            return rgb_img, depth_img
    
    def world_pos2img_pos(self, xs_world: np.ndarray, rotation: np.ndarray, translation: np.ndarray):
        """ Convert world positions to image position
        Args:
            xs_world (shape=(N,3)): World positions
            rotation (shape=(3,3)): Rotation matrix (World to Camera coord-sys)
            translation (shape=(3,)): Translation matrix (World to Camera coord-sys)
        Returns:
            xs_img (shape=(N,2)): Width and Height
        """
        if xs_world.ndim == 1:
            xs_world = xs_world.reshape(1, -1)

        T = translation.reshape(3, 1)
        R = rotation
        xs_camera = xs_world @ R.T + T.T
        xs_img = xs_camera @ self.K.T / xs_camera[:, -1:]
        xs_img = xs_img[:, :2].astype(np.int32)
        return xs_img
    
    def img_pos2world_pos(self, xs_img: np.ndarray, depth_img: np.ndarray, rotation: np.ndarray, translation: np.ndarray):
        """ Convert image positions to world position
        Args:
            xs_img (shape=(N,2)): Image positions (W, H)
            depth_img (shape=(H,W)): Depth image
            rotation (shape=(3,3)): Rotation matrix (World to Camera coord-sys)
            translation (shape=(3,)): Translation matrix (World to Camera coord-sys)
        """
        xs_img = xs_img.copy()
        if xs_img.ndim == 1:
            xs_img = xs_img.reshape(1, -1)
        N = xs_img.shape[0]
        xs_img[:, 0] = np.clip(xs_img[:, 0], 0, self.width-1)
        xs_img[:, 1] = np.clip(xs_img[:, 1], 0, self.height-1)
        xs_img = np.concatenate([xs_img, np.ones((N, 1), xs_img.dtype)], axis=-1)

        # Image to Camera coord-sys
        K_inv = np.linalg.inv(self.K)
        xs = xs_img @ K_inv.T
        r2 = xs[:, 0] ** 2 + xs[:, 1] ** 2
        k1, k2, p1, p2 = self.dist
        x, y = xs[:, 0], xs[:, 1]
        xs[:, 0] = x * (1+k1*r2+k2*r2**2) + 2*p1*x*y + p2*(r2+2*x**2)
        xs[:, 1] = y * (1+k1*r2+k2*r2**2) + p1*(r2+2*y**2) + 2*p2*x*y

        xs_img_idxs = xs_img.astype(np.int32)
        xs_depth = depth_img[xs_img_idxs[:,1], xs_img_idxs[:,0]] / 1e3  # NOTE: millimeter -> meter
        xs_camera = xs * xs_depth.reshape(-1, 1)

        # Camera to World corrd-sys
        T = translation.reshape(3, 1)
        R = rotation
        xs_world = (xs_camera - T.T) @ R  # NOTE: R.inv.T == R
        return xs_world
    
def debug_camera():
    camera = D435iCamera()
    while True:
        rgb_img, depth_img = camera.get_frame()
        depth_normalized = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
        depth_normalized = depth_normalized.astype(np.uint8)
        print(rgb_img.shape, depth_img.min(), depth_img.max())
        cv2.imshow("debug rgb", rgb_img)
        cv2.imshow("debug depth", depth_normalized)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    debug_camera()
