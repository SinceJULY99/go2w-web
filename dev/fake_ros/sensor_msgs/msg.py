from types import SimpleNamespace as N


class PointCloud2:
    """只填 go2w_web 读的字段：x / y / z / intensity 四个 float32，16 字节一个点。"""

    def __init__(self, xyz=None, frame='body'):
        import numpy as np
        pts = np.zeros((0, 4), dtype=np.float32) if xyz is None else np.hstack([xyz, np.zeros((len(xyz), 1))]).astype(np.float32)
        self.header = N(frame_id=frame, stamp=N(sec=0, nanosec=0))
        self.height, self.width = 1, len(pts)
        self.fields = [N(name=n, offset=4*i, datatype=7, count=1) for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
        self.point_step, self.row_step = 16, 16 * len(pts)
        self.is_bigendian, self.is_dense = False, True
        self.data = pts.tobytes()


class LaserScan:
    def __init__(self):
        self.header = N(frame_id='', stamp=N(sec=0, nanosec=0))
        self.angle_min = self.angle_increment = 0.0
        self.range_min, self.range_max = 0.1, 15.0
        self.ranges = []
