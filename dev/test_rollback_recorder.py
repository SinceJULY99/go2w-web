#!/usr/bin/env python3
"""回溯录制的缓存和存包逻辑（不需要 ROS）。python3 dev/test_rollback_recorder.py"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'scripts'))
from rollback_recorder import Ring, save  # noqa: E402

S = 1_000_000_000
fails = 0


def check(name, ok, info=''):
    global fails
    fails += not ok
    print(('ok  ' if ok else 'FAIL'), name, info if not ok else '')


r = Ring(20, 10**9)
r.add(0, '/map', b'map', latched=True)
r.add(0, '/tf_static', b'a', latched=True)
r.add(1, '/tf_static', b'b', latched=True)
for i in range(60):
    r.add((i + 2) * S, '/odom', b'x' * 10)
snap = r.snapshot()
odom = [x for x in snap if x[1] == '/odom']
check('只留最近 20 秒', odom[0][0] == 41 * S and odom[-1][0] == 61 * S, odom[0])
check('字节计数和缓存一致', r.bytes == sum(len(x[2]) for x in r.items), r.bytes)
check('地图过期后仍然保留', [x[2] for x in snap if x[1] == '/map'] == [b'map'], snap[:3])
check('静态变换各留一份', sorted(x[2] for x in snap if x[1] == '/tf_static') == [b'a', b'b'])
check('保留的消息放在窗口开头', all(x[0] == 41 * S for x in snap if x[1] in ('/map', '/tf_static')))
check('时间单调', [x[0] for x in snap] == sorted(x[0] for x in snap))

r = Ring(20, 100)
for i in range(30):
    r.add(i, '/lidar', b'y' * 10)
check('超过字节上限丢最旧的', r.bytes <= 100 and len(r.items) == 10, r.bytes)

r = Ring(20, 10**9)
r.add(5 * S, '/map', b'old', latched=True)
r.add(6 * S, '/map', b'new', latched=True)
r.add(7 * S, '/odom', b'o')
check('窗口内的地图不重复，且窗口外的被新的替换', [x[2] for x in r.snapshot() if x[1] == '/map'] == [b'old', b'new'], r.snapshot())
r.add(30 * S, '/odom', b'o')
check('窗口外只留最新地图', [x[2] for x in r.snapshot() if x[1] == '/map'] == [b'new'], r.snapshot())

with tempfile.TemporaryDirectory() as d:
    os.makedirs(os.path.join(d, '.rollback'))

    def write(path, records, types):
        os.makedirs(path)
        open(os.path.join(path, 'metadata.yaml'), 'w').close()

    check('保存成功', save('snap_a', [(1, '/odom', b'x')], {'/odom': 't'}, d, write))
    check('包在 bags 目录下', os.path.exists(os.path.join(d, 'snap_a', 'metadata.yaml')))
    check('不留临时目录', not os.path.exists(os.path.join(d, '.rollback', 'snap_a')))
    check('空缓存报错', not save('snap_b', [], {}, d, write) and os.path.exists(os.path.join(d, '.rollback', 'snap_b.error')))
    check('不覆盖已有的包', not save('snap_a', [(1, '/odom', b'x')], {'/odom': 't'}, d, write))

    def boom(path, records, types):
        os.makedirs(path)
        raise RuntimeError('disk full')

    check('写失败返回 False', not save('snap_c', [(1, '/odom', b'x')], {'/odom': 't'}, d, boom))
    check('写失败不留半个包', not os.path.exists(os.path.join(d, 'snap_c')) and not os.path.exists(os.path.join(d, '.rollback', 'snap_c')))
    with open(os.path.join(d, '.rollback', 'snap_c.error')) as f:
        check('错误写进 .error', f.read() == 'disk full')

print('全部通过' if not fails else f'{fails} 项失败')
sys.exit(1 if fails else 0)
