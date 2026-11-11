"""分段录包 scripts/segment_recorder.py 的切段逻辑（不需要 ROS）。

    python3 web/dev/test_segment_recorder.py
"""
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'scripts'))
from segment_recorder import Segments, recover_leftovers  # noqa: E402

fails = 0


def check(name, ok, extra=''):
    global fails
    fails += not ok
    print(('PASS ' if ok else 'FAIL ') + name, extra)


class FakeWriter:
    """写到一个 records.txt 里，close 时写 metadata.yaml（和 rosbag2 一样，metadata 出现 = 写完）。"""
    def __init__(self, path):
        os.makedirs(path)
        self.path, self.rows, self.topics = path, [], []

    def create_topic(self, topic, type_str):
        self.topics.append(topic)

    def write(self, topic, data, t):
        self.rows.append((t, topic, data))

    def close(self):
        with open(os.path.join(self.path, 'records.txt'), 'w') as f:
            for t, topic, data in self.rows:
                f.write(f'{t} {topic} {data}\n')
        open(os.path.join(self.path, 'metadata.yaml'), 'w').close()


def rows(bags, name):
    with open(os.path.join(bags, name, 'records.txt')) as f:
        return [line.split() for line in f]


S = 1_000_000_000
d = tempfile.mkdtemp()
clock = [0]
seg = Segments(d, 'web', 30, FakeWriter, log=lambda *a, **k: None, clock=lambda: clock[0], local=time.gmtime)

base = 1_762_830_000 * S          # 2025-11-11 03:00:00 UTC
seg.add('/tf_static', 'tf2_msgs/msg/TFMessage', 'tfs', latched=True, t=base)
seg.add('/map', 'nav_msgs/msg/OccupancyGrid', 'map1', latched=True, t=base + 1)
for k in range(0, 75):            # 75 s，每秒一条 /odom；第 50 s 地图更新一次
    if k == 50:
        seg.add('/map', 'nav_msgs/msg/OccupancyGrid', 'map2', latched=True, t=base + 50 * S)
    seg.add('/odom', 'nav_msgs/msg/Odometry', f'o{k}', t=base + k * S + 2)
clock[0] = base + 75 * S
seg.stop()

names = sorted(n for n in os.listdir(d) if not n.startswith('.'))
check('75 s 切成 3 个包', len(names) == 3, names)
t0 = time.strftime('%m%d_%H%M%S', time.gmtime(base / S))
check('包名 = 前缀_MMDD_HHMMSS（这一段开始时间）',
      names == [f'web_{t0}', f'web_{time.strftime("%m%d_%H%M%S", time.gmtime(base / S + 30))}',
                f'web_{time.strftime("%m%d_%H%M%S", time.gmtime(base / S + 60))}'], names)
check('.recording 里不留东西', os.listdir(os.path.join(d, '.recording')) == [])
odom = [r[2] for n in names for r in rows(d, n) if r[1] == '/odom']
check('段与段之间不丢、不重复消息', odom == [f'o{k}' for k in range(75)], len(odom))
for i, n in enumerate(names):
    rs = rows(d, n)
    heads = {r[1] for r in rs[:2]}
    check(f'第 {i + 1} 段开头有 /tf_static 和 /map', heads == {'/tf_static', '/map'}, [r[1] for r in rs[:3]])
second, third = rows(d, names[1]), rows(d, names[2])
check('段中途更新的 /map 写进当前段', any(r[1] == '/map' and r[2] == 'map2' for r in second))
check('下一段开头用最新的 /map', [r[2] for r in third if r[1] == '/map'][0] == 'map2')
check('每段时长不超过 30 s', all(int(rows(d, n)[-1][0]) - int(rows(d, n)[0][0]) < 30 * S for n in names))

# 没有消息时，tick 也按时切段
d2 = tempfile.mkdtemp()
clock[0] = base
seg = Segments(d2, 'x', 30, FakeWriter, log=lambda *a, **k: None, clock=lambda: clock[0], local=time.gmtime)
seg.add('/odom', 'nav_msgs/msg/Odometry', 'a', t=base)
clock[0] = base + 31 * S
seg.tick()
seg.stop()
check('没消息时 tick 也切段（前一段已存好）', len([n for n in os.listdir(d2) if not n.startswith('.')]) >= 1)

# 同一秒重名时加后缀
d3 = tempfile.mkdtemp()
os.makedirs(os.path.join(d3, f'web_{t0}'))
seg = Segments(d3, 'web', 30, FakeWriter, log=lambda *a, **k: None, clock=lambda: base, local=time.gmtime)
seg.add('/odom', 'nav_msgs/msg/Odometry', 'a', t=base)
seg.stop()
check('重名时加 _2', os.path.isdir(os.path.join(d3, f'web_{t0}_2')))

# 上次异常退出留下的段
d4 = tempfile.mkdtemp()
os.makedirs(os.path.join(d4, '.recording', 'web_1111_120000'))
recover_leftovers(d4, log=lambda *a, **k: None)
check('残留的段移到 data/bags，名字带 _unfinished', os.path.isdir(os.path.join(d4, 'web_1111_120000_unfinished')))

print('全部通过' if not fails else f'{fails} 项失败')
sys.exit(1 if fails else 0)
