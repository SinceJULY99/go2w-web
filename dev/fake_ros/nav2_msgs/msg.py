import math
from types import SimpleNamespace as N


class ParticleCloud:
    def __init__(self, particles=(), frame='map'):
        # particles: [(x, y, yaw, weight)]
        self.header = N(frame_id=frame, stamp=N(sec=0, nanosec=0))
        self.particles = [N(pose=N(position=N(x=x, y=y, z=0.0), orientation=N(x=0.0, y=0.0, z=math.sin(a/2), w=math.cos(a/2))),
                            weight=w) for x, y, a, w in particles]


class BehaviorTreeLog:
    def __init__(self, events=()):
        # events: [(node_name, previous_status, current_status)]
        self.event_log = [N(node_name=n, previous_status=p, current_status=c) for n, p, c in events]
