from types import SimpleNamespace as N


class OccupancyGrid:
    def __init__(self):
        self.header = N(frame_id='map')
        self.info = N(width=0, height=0, resolution=0.05,
                      origin=N(position=N(x=0.0, y=0.0, z=0.0), orientation=N(x=0.0, y=0.0, z=0.0, w=1.0)))
        self.data = []


class Path:
    def __init__(self, pts=(), frame='map'):
        self.header = N(frame_id=frame, stamp=N(sec=0, nanosec=0))
        self.poses = [N(pose=N(position=N(x=x, y=y, z=0.0), orientation=N(x=0.0, y=0.0, z=0.0, w=1.0))) for x, y in pts]


class Odometry:
    def __init__(self):
        self.header = N(frame_id='odom', stamp=N(sec=0, nanosec=0))
        self.child_frame_id = 'base_footprint'
        self.pose = N(pose=N(position=N(x=0.0, y=0.0, z=0.0), orientation=N(x=0.0, y=0.0, z=0.0, w=1.0)))
        self.twist = N(twist=N(linear=N(x=0.0, y=0.0, z=0.0), angular=N(x=0.0, y=0.0, z=0.0)))
