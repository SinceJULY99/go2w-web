"""卡住自动保存：StuckWatch 判定 + Server.stuck_tick 的触发条件。

    ~/.cache/go2w_web_venv/bin/python web/dev/test_stuck.py
"""
import asyncio
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, 'fake_ros'), os.path.join(HERE, '..', 'server')]
import go2w_web as W  # noqa: E402

fails = 0


def check(name, ok):
    global fails
    fails += not ok
    print(('PASS ' if ok else 'FAIL ') + name)


def watch():
    w = W.StuckWatch(seconds=8, move=0.08, turn=0.15)
    check('开始计时不报', w.update(0, (0, 0, 0), True) is None)
    check('7.9 s 不报', w.update(7.9, (0.05, 0, 0.1), True) is None)
    r = w.update(8.0, (0.05, 0, 0.1), True)
    check('8 s 报一次', r is not None and abs(r - 8) < 1e-9)
    check('同一次停住只报一次', w.update(20, (0, 0, 0), True) is None)
    check('动了重新计时', w.update(21, (0.2, 0, 0), True) is None and w.update(28.9, (0.2, 0, 0), True) is None)
    check('再停 8 s 又报', w.update(29.1, (0.2, 0, 0), True) is not None)
    w = W.StuckWatch(seconds=8, move=0.08, turn=0.15)
    w.update(0, (0, 0, 3.1), True)
    check('原地转（跨 ±π）算在动', w.update(5, (0, 0, -2.9), True) is None and w.update(12, (0, 0, -2.9), True) is None)
    check('原地小抖不算动', w.update(13.1, (0.01, 0, -2.95), True) is not None)
    w = W.StuckWatch(seconds=8, move=0.08, turn=0.15)
    w.update(0, (0, 0, 0), True)
    w.update(5, (0, 0, 0), False)
    check('任务结束后清零', w.update(6, (0, 0, 0), True) is None and w.update(13, (0, 0, 0), True) is None)
    check('没有 odom 不报', W.StuckWatch(seconds=0).update(1, None, True) is None)


async def tick():
    events, saves = [], []
    t0 = W.time.monotonic()
    node = types.SimpleNamespace(odom_pose=(1.0, 2.0, 0.0, t0), nav_active=lambda: True, is_armed=lambda: True,
                                 event=lambda lv, msg: events.append((lv, msg)))
    s = types.SimpleNamespace(node=node, estop=False, procs_status={'rollback': True}, rollback_saving=False,
                              stuck=W.StuckWatch(seconds=0.2), stuck_t=-W.STUCK_COOLDOWN, pending=set())
    s.background = lambda c: W.Server.background(s, c)

    async def save(who, seconds=None):
        saves.append(who)
        secs.append(seconds)
    secs = []
    s.rollback_save = save
    W.STUCK_AFTER = 0.1
    tick = lambda: W.Server.stuck_tick(s)
    tick()
    await asyncio.sleep(0.25)
    node.odom_pose = (1.0, 2.0, 0.0, W.time.monotonic())
    tick()
    await asyncio.sleep(0.2)
    check('卡住后自动保存', saves == ['卡住自动保存'])
    check(f'存 前 15 秒 + 后 0.1 秒 {secs}', secs == [W.STUCK_BEFORE + 0.1])
    check('先报警告', any(lv == 'WARN' and '没动' in m for lv, m in events))
    # 冷却期内再次卡住：只报警告不存
    s.stuck = W.StuckWatch(seconds=0.2)
    tick(); await asyncio.sleep(0.25); node.odom_pose = (1.0, 2.0, 0.0, W.time.monotonic()); tick()
    await asyncio.sleep(0.2)
    check('冷却期内不重复存', len(saves) == 1 and '这次不存' in events[-1][1])
    # 没解锁 / 急停 / odom 过期都不算
    for name, setup in [('上锁不算', lambda: setattr(node, 'is_armed', lambda: False)),
                        ('急停不算', lambda: setattr(s, 'estop', True)),
                        ('odom 过期不算', lambda: setattr(node, 'odom_pose', (1.0, 2.0, 0.0, W.time.monotonic() - 10)))]:
        node.is_armed, s.estop, s.stuck_t, n0 = (lambda: True), False, -W.STUCK_COOLDOWN, len(saves)
        s.stuck = W.StuckWatch(seconds=0.2)
        node.odom_pose = (1.0, 2.0, 0.0, W.time.monotonic())
        setup()
        tick(); await asyncio.sleep(0.25)
        if name != 'odom 过期不算':
            node.odom_pose = (1.0, 2.0, 0.0, W.time.monotonic())
        tick(); await asyncio.sleep(0.2)
        check(name, len(saves) == n0)
    # 回溯没在跑：只报警告
    node.is_armed, s.estop, s.stuck_t, s.procs_status = (lambda: True), False, -W.STUCK_COOLDOWN, {}
    s.stuck = W.StuckWatch(seconds=0.2)
    node.odom_pose = (1.0, 2.0, 0.0, W.time.monotonic()); tick(); await asyncio.sleep(0.25)
    node.odom_pose = (1.0, 2.0, 0.0, W.time.monotonic()); tick(); await asyncio.sleep(0.2)
    check('回溯没在跑时只报警告', len(saves) == 1 and '回溯录制没在运行' in events[-1][1])


watch()
asyncio.run(tick())
print('全部通过' if not fails else f'{fails} 项失败')
sys.exit(1 if fails else 0)
