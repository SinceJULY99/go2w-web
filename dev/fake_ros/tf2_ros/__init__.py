import math
from types import SimpleNamespace as N

BODY_Z = 0.53   # 假 FAST-LIO 的 body 坐标系：在 base_footprint 正上方（真机上还有约 16° 俯仰）


def _tf(x=0.0, y=0.0, z=0.0, q=None):
    q = q or N(x=0.0, y=0.0, z=0.0, w=1.0)
    return N(transform=N(translation=N(x=x, y=y, z=z), rotation=q))


class Buffer:
    """假 TF：map 和 odom 重合（假里程计没有漂移），机器人位姿取自假模拟器。"""

    def __init__(self):
        self.node = None

    def lookup_transform(self, fixed, frame, _t):
        sim = getattr(self.node, 'fake_sim', None)
        if sim is None or not sim.any_ready():   # 导航栈没跑时没有 TF
            raise LookupError('no tf')
        if fixed in ('map', 'odom') and frame in ('map', 'odom'):
            return _tf()
        if fixed == 'base_footprint' and frame == 'base_link':
            # planar_odom 发的机身姿态：加减速时轻微俯仰，转弯时轻微横滚
            roll, pitch = sim.tilt()
            cr, sr, cp, sp = math.cos(roll/2), math.sin(roll/2), math.cos(pitch/2), math.sin(pitch/2)
            return _tf(q=N(x=sr*cp, y=cr*sp, z=-sr*sp, w=cr*cp))
        if fixed == 'base_footprint' and frame == 'body':
            return _tf(z=BODY_Z)
        if fixed in ('map', 'odom') and frame == 'base_footprint':
            return _tf(sim.x, sim.y, 0.0, N(x=0.0, y=0.0, z=math.sin(sim.th / 2), w=math.cos(sim.th / 2)))
        raise LookupError(f'{fixed} -> {frame} not available')


class TransformListener:
    def __init__(self, buf, node):
        buf.node = node
