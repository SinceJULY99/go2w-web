from types import SimpleNamespace as N


def _pose():
    return N(position=N(x=0.0, y=0.0, z=0.0), orientation=N(x=0.0, y=0.0, z=0.0, w=1.0))


class Twist:
    def __init__(self):
        self.linear = N(x=0.0, y=0.0, z=0.0)
        self.angular = N(x=0.0, y=0.0, z=0.0)


class PoseStamped:
    def __init__(self):
        self.header = N(frame_id='', stamp=N(sec=0, nanosec=0))
        self.pose = _pose()


class PoseWithCovarianceStamped:
    def __init__(self):
        self.header = N(frame_id='', stamp=N(sec=0, nanosec=0))
        self.pose = N(pose=_pose(), covariance=[0.0] * 36)
