#!/usr/bin/env python3
"""回溯录制的缓存和存包逻辑（不需要 ROS）。python3 web/dev/test_rollback_recorder.py"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'scripts'))
from rollback_recorder import Ring, save, wanted  # noqa: E402

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

    check('保存成功', save('go2w_a', [(1, '/odom', b'x')], {'/odom': 't'}, d, write))
    check('包在 bags 目录下', os.path.exists(os.path.join(d, 'go2w_a', 'metadata.yaml')))
    check('不留临时目录', not os.path.exists(os.path.join(d, '.rollback', 'go2w_a')))
    check('空缓存报错', not save('go2w_b', [], {}, d, write) and os.path.exists(os.path.join(d, '.rollback', 'go2w_b.error')))
    check('不覆盖已有的包', not save('go2w_a', [(1, '/odom', b'x')], {'/odom': 't'}, d, write))

    def boom(path, records, types):
        os.makedirs(path)
        raise RuntimeError('disk full')

    check('写失败返回 False', not save('go2w_c', [(1, '/odom', b'x')], {'/odom': 't'}, d, boom))
    check('写失败不留半个包', not os.path.exists(os.path.join(d, 'go2w_c')) and not os.path.exists(os.path.join(d, '.rollback', 'go2w_c')))
    with open(os.path.join(d, '.rollback', 'go2w_c.error')) as f:
        check('错误写进 .error', f.read() == 'disk full')

r = Ring(20, 1000)
for i in range(30):
    r.add(i * 10**9, '/x', b'a' * 10)
check('不超上限时按 20 秒保留', abs(r.span() - 20) < 1e-9 and r.capped == 0, (r.span(), r.capped))
r = Ring(20, 100)
for i in range(30):
    r.add(i * 10**9, '/x', b'a' * 10)
check('超上限时截短并计数', r.span() < 20 and r.capped > 0 and r.bytes <= 100, (r.span(), r.capped, r.bytes))


r = Ring(30, 10**9)
r.add(0, '/map', b'map', latched=True)
for i in range(31):
    r.add((i + 1) * S, '/odom', b'o')
snap = r.snapshot(20)
odom = [x for x in snap if x[1] == '/odom']
check('snapshot(20) 只取最近 20 秒', odom[0][0] == 11 * S and odom[-1][0] == 31 * S, (odom[0][0], odom[-1][0]))
check('snapshot(20) 地图放在开头', snap[0] == (11 * S, '/map', b'map'), snap[0])
check('snapshot() 取整个缓存', [x for x in r.snapshot() if x[1] == '/odom'][0][0] == 1 * S)


class FakeNode:
    def get_topic_names_and_types(self):
        return [('/scan', ['t']), ('/odom', ['t']), ('/nav/_action/status', ['t'])]


check('只录列出的', wanted(FakeNode(), ['/odom'], False) == ['/odom'])
check('--all 录全部，列出的在前不重复', wanted(FakeNode(), ['/odom'], True) == ['/odom', '/nav/_action/status', '/scan'])
check('--all 不列也行', set(wanted(FakeNode(), [], True)) == {'/scan', '/odom', '/nav/_action/status'})

print('全部通过' if not fails else f'{fails} 项失败')
sys.exit(1 if fails else 0)
