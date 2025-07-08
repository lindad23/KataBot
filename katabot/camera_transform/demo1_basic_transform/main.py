import numpy as np
import cv2
from typing import Sequence

class SpaceObject:
    x_world: np.ndarray
    lines: np.ndarray

class Cube(SpaceObject):
    def __init__(self, width=1, center=np.array([0,0,0])):
        self.width = width
        self.center = center
        self.x_world = (np.array([
            [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1],
            [1, 1, 0], [1, 0, 1], [0, 1, 1], [1, 1, 1],
        ], np.float32) - np.array([0.5]*3, np.float32)) * width + np.ones(3, np.float32) * center
        self.lines = np.array([
            [0, 1], [0, 2], [1, 4], [2, 4],
            [0, 3], [1, 5], [2, 6], [4, 7],
            [5, 7], [6, 7], [3, 6], [3, 5],
        ], np.int32)

class Triangle(SpaceObject):
    def __init__(self, pos=None):
        if pos is None:
            pos = [(1,0,0), (0,1,0), (0,0,0)]
        self.x_world = np.array(pos, np.float32)    # (3, 3)
        self.lines = np.array([[0, 1], [1, 2], [2, 0]], np.int32)

def rotation_3d(rot: Sequence):
    rx, ry, rz = rot
    sin, cos = np.sin, np.cos
    Rx = np.array([[1,0,0],[0,cos(rx),-sin(rx)],[0,sin(rx),cos(rx)]], np.float32)
    Ry = np.array([[cos(ry),0,sin(ry)],[0,1,0],[-sin(ry),0,cos(ry)]], np.float32)
    Rz = np.array([[cos(rz),-sin(rz),0],[sin(rz),cos(rz),0],[0,0,1]], np.float32)
    return Rx @ Ry @ Rz

class Camera:
    def __init__(self, fx=800, fy=800, u0=256, v0=256, rot=[np.pi/4,np.pi/6,np.pi/4], tran=[0,0,5], name='camera'):
        self.name = name
        self.fx, self.fy, self.u0, self.v0 = fx, fy, u0, v0
        self.img_size = (self.u0 * 2, self.v0 * 2)
        self.K = np.array([
            [fx, 0, u0],
            [0, fy, v0],
            [0, 0, 1]
        ], np.float32)
        self.rot = np.array(rot, np.float32)
        self.R = rotation_3d(rot)
        self.T = np.array(tran, dtype=np.float32)
        self.x_camera, self.x_img = [], []

    def cap(self, obj: SpaceObject, draw_vertex=True, show=True, delay=10, img=None):
        x_world = obj.x_world    # (N, 3)
        x_camera = x_world @ self.R.T + self.T.T
        self.x_camera.append(x_camera)
        x_img = x_camera @ self.K.T / x_camera[:, -1:]
        x_img = x_img[:, :2].astype(np.int32)
        self.x_img.append(x_img)
        if img is None:
            img = np.zeros((*self.img_size, 3), np.uint8)
        for (i, j) in obj.lines:
            cv2.line(img, x_img[i], x_img[j], color=(255,192,203), thickness=2)
        if draw_vertex:
            for pos in x_img:
                cv2.circle(img, pos, radius=1, color=(255,0,0), thickness=4)
        if show: self.show(img, delay)
        return img

    def add_objs(self, objs: Sequence[SpaceObject], delay=10, show=True):
        img = None
        self.x_img = []
        for obj in objs:
            img = self.cap(obj, draw_vertex=False, show=False, img=img)
        self.x_img = np.concatenate(self.x_img, 0)
        if show:
            self.show(img, delay)
        return img

    def show(self, img, delay):
        cv2.imshow(self.name, img)
        cv2.waitKey(delay)

def get_extrinsics_matrix_pnp(x_img: np.ndarray, x_world: np.ndarray, K: np.ndarray, dist=None):
    """ Give one-to-one N (N >= 4) points in world and image coordinate systems, return world coor-sys to camera coor-sys.
    Args:
        x_img: [shape=(N,2)] Points in image coordinate system
        x_world: [shape=(N,3)] Points in world coordinate system
        K: [shape=(3,3)] Camera internal matrix
        dist: [shape=(4,) or None] If not None, give distortion coefficients k1, k2, p1, p2 (option: [k3, k4] after)
    Returns: (World coor-sys to Camera coor-sys)
        R: [shape=(3,3)] Rotation matrix
        T: [shape=(3,)] Translation vector

    """
    assert len(x_img) == len(x_world) and len(x_img) >= 4
    _, R, T = cv2.solvePnP(
        x_world.astype(np.float32), x_img.astype(np.float32), K.astype(np.float32), distCoeffs=dist,
        flags=cv2.SOLVEPNP_ITERATIVE  # more stable
    )
    R, _ = cv2.Rodrigues(R)
    return R, T[:, 0]

def get_extrinsics_matrix_oxy(o: np.ndarray, x: np.ndarray, y: np.ndarray):
    """ Give 3 points in world and camera coordinate systems, return world coor-sys to camera coor-sys.
    Args:
        o: [shape=(3,)] World (0, 0, 0) in camera coordinate system
        x: [shape=(3,)] World (x, 0, 0) in camera coordinate system, x can be any real number
        y; [shape=(3,)] World (0, y, 0) in camera coordinate system, y can be any real number
    Returns: (World coor-sys to Camera coor-sys)
        R: [shape=(3,3)] Rotation matrix
        T: [shape=(3,)] Translation vector
    """
    T = o
    r1, r2 = (x - T) / np.linalg.norm(x - T), (y - T) / np.linalg.norm(y - T)
    r3 = np.cross(r1, r2)
    r3 /= np.linalg.norm(r3)
    R = np.array([r1,r2,r3], np.float32).T
    return R, T

def random_rotation(camera: Camera, objs, n_sample=100, rand_range=2*np.pi, rand_time=np.inf):
    camera.add_objs(objs)

    rot = base_rot = camera.rot
    i = 0
    while i < rand_time:
        i += 1
        # target = [rand[0], np.pi, np.pi]
        delta = (np.random.rand(3) - 0.5) * rand_range / 2
        target = base_rot + delta
        rs = np.linspace(rot, target, n_sample)
        for r in rs:
            camera.R = rotation_3d(r)
            camera.add_objs(objs)
        rot = target
        # print(r.shape)

def demo1_play_with_graphics():
    camera = Camera(name='cube')
    objs = [Cube(),
        Triangle([(0,0,0),(1,0,0),(0,0,1)]), Triangle([(0,0,0),(-1,0,0),(0,0,1)]),
        Triangle([(0,0,0),(1,0,0),(0,0,-1)]), Triangle([(0,0,0),(-1,0,0),(0,0,-1)]),
        Triangle([(0,0,0),(0,1,0),(0,0,1)]), Triangle([(0,0,0),(0,1,0),(0,0,-1)]),
        Triangle([(0,0,0),(0,-1,0),(0,0,1)]), Triangle([(0,0,0),(0,-1,0),(0,0,-1)]),
    ]
    random_rotation(camera, objs)

def demo2_play_with_text():
    from stl import mesh
    camera = Camera(v0=512, rot=[0,np.pi,np.pi], tran=[10,-10,30], name='text')
    ms = mesh.Mesh.from_file('test1.stl')
    objs = [Triangle(tri) for tri in ms.vectors]
    random_rotation(camera, objs, n_sample=10, rand_range=np.pi/2)

def demo3_get_extrinsics_matrix_pnp():
    camera = Camera(name='cube')
    obj = Cube()
    camera.add_objs([obj], show=False)
    R, T = get_extrinsics_matrix_pnp(camera.x_img, obj.x_world, camera.K)
    print("(DEMO3) Calculate extrinsics by N image and world points relation (PnP):")
    print("real:", camera.R, camera.T)
    print("calc:", R, T); print()

def demo4_get_extrinsics_matrix_oxy():
    camera = Camera(name='cube')
    x_world = np.array([[0,0,0], [3,0,0], [0,-10,0]], np.float32)
    x_camera = x_world @ camera.R.T + camera.T.T
    R, T = get_extrinsics_matrix_oxy(x_camera[0], x_camera[1], x_camera[2])
    print("(DEMO4) Calculate extrinsics by 3 point world (origin point, x axis point, y axis point) in image coordinate:")
    print("real:", camera.R, camera.T)
    print("calc:", R, T); print()

if __name__ == '__main__':
    # demo1_play_with_graphics()
    demo3_get_extrinsics_matrix_pnp()
    # demo4_get_extrinsics_matrix_oxy()
    # import multiprocessing
    # multiprocessing.Process(target=demo1_play_with_graphics).start()
    # demo2_play_with_text()    # add test1.stl in current folder, generate by https://www.enjoying3d.com/tool/text.php