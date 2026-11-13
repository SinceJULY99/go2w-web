#!/usr/bin/env python3
"""RuntimeProcs 测试：起一个假的 supervisor socket，检查请求格式、状态换算、参数校验和出错处理。

本地模拟器用 FakeProcs，绕过了 supervisor；这里验证 server/procs.py 和运行容器 supervisor 之间的协议。
存图 / 对齐地图是网页自己起的子进程，用临时的 tools 目录里的假脚本测。
运行：python3 dev/test_procs.py
"""
import asyncio, json, os, sys, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'server'))
import procs as P

requests = []
groups = {}        # 假 supervisor 的进程组：名字 → {'running', 'environment'}
reply = None       # 不是 None 时原样回这一行（测出错处理）


async def handle(reader, writer):
    line = await reader.readline()
    req = json.loads(line); requests.append(req)
    if reply is not None:
        writer.write(reply)
    else:
        op, name = req['operation'], req.get('name')
        if op == 'status':
            result = groups
        elif op == 'start':
            groups[name] = {'running': True, 'environment': req.get('environment', {})}
            result = groups[name]
        elif op == 'stop':
            groups.setdefault(name, {})['running'] = name == 'stuck'
            result = groups[name]
        writer.write(json.dumps({'ok': True, 'result': result}).encode() + b'\n')
    await writer.drain(); writer.close()


async def main():
    global reply
    tmp = tempfile.mkdtemp()
    sock = os.path.join(tmp, 's.sock')
    server = await asyncio.start_unix_server(handle, sock)
    tools = os.path.join(tmp, 'tools'); os.makedirs(tools)
    with open(os.path.join(tools, 'save_map.sh'), 'w') as f:
        f.write('[ "$1" = good ] && exit 0; exit 3\n')
    with open(os.path.join(tools, 'align_map.py'), 'w') as f:
        f.write('import sys; print(sys.argv[1:]); sys.exit(0)\n')
    d, fails = P.RuntimeProcs(sock=sock, data_dir=os.path.join(tmp, 'data'), tools_dir=tools), []

    def check(name, cond, extra=''):
        print(('PASS' if cond else 'FAIL'), name, extra); fails.append(name) if not cond else None

    check('socket 存在时 available', d.available())
    check('socket 不存在时不 available', not P.RuntimeProcs(sock=os.path.join(tmp, 'none.sock')).available())

    st = await d.status(); check('没有进程组时全部 false', not any(st.values()), st)
    check('状态请求格式', requests[-1] == {'operation': 'status'}, requests[-1])

    await d.start_mode('localization', 'lab')
    check('启动档位走 navigation 组', requests[-1] == {'operation': 'start', 'name': 'navigation',
          'environment': {'GO2W_NAVIGATION_MODE': 'localization', 'GO2W_MAP': 'lab.yaml'}}, requests[-1])
    st = await d.status(); check('按 GO2W_NAVIGATION_MODE 识别档位', st['localization'] and not st['navigation'] and not st['mapping'], st)
    await d.start_mode('mapping')
    check('建图不带地图', requests[-1]['environment'] == {'GO2W_NAVIGATION_MODE': 'mapping'}, requests[-1])

    await d.start_patrol(3); await d.start_bag('go2w', ['*', '/odom']); await d.start_rollback(20); await d.start_bridge()
    st = await d.status()
    check('识别巡逻 / 录包 / 回溯 / 速度桥', st['patrol'] and st['bag'] and st['bag_seg'] and st['rollback'] and st['bridge'], st)
    check('录包参数', groups['bag']['environment'] == {'GO2W_BAG_PREFIX': 'go2w', 'GO2W_BAG_TOPICS': '["*","/odom"]',
          'GO2W_BAG_SECONDS': '30.0'}, groups['bag']['environment'])
    check('回溯参数', groups['rollback']['environment'] == {'GO2W_ROLLBACK_SECONDS': '20.0'}, groups['rollback']['environment'])

    n0 = len(requests); ok = await d.stop_modes()
    stopped = [r['name'] for r in requests[n0:]]
    check('stop_modes 只停巡逻、导航、速度桥', ok and stopped == ['patrol', 'navigation', 'bridge'], stopped)
    st = await d.status(); check('stop_modes 不影响录包和回溯', st['bag'] and st['rollback'] and not st['patrol'], st)
    groups['stuck'] = {'running': True}
    check('组没停掉时如实报告失败', await d._stop('stuck') is False)

    bad_calls = (
        ('名字 a;b', lambda: d.start_bag('a;b', ['/odom'])),
        ('地图 $(id)', lambda: d.start_mode('navigation', '$(id)')),
        ('未知档位', lambda: d.start_mode('rm', 'lab')),
        ('topic **', lambda: d.start_bag('t', ['**'])),
        ('topic *;id', lambda: d.start_bag('t', ['*;id'])),
        ('空 topic 列表', lambda: d.start_bag('t', [])),
        ('时长 0', lambda: d.start_rollback(0)),
        ('时长 nan', lambda: d.start_rollback(float('nan'))),
        ('巡逻圈数 0', lambda: d.start_patrol(0)),
        ('巡逻圈数 True', lambda: d.start_patrol(True)),
        ('存图名 x y', lambda: d.save_map('x y')),
        ('对齐源 = 目标', lambda: d.align_map('a', 'a', 0.1)),
        ('对齐角度 2 rad', lambda: d.align_map('a', 'b', 2.0)),
    )
    for name, fn in bad_calls:
        n0 = len(requests)
        try:
            await fn(); check(f'拒绝{name}', False)
        except ValueError:
            check(f'拒绝{name}', len(requests) == n0)

    check('存图成功', await d.save_map('good'))
    check('存图失败返回 False', not await d.save_map('bad'))
    check('对齐地图', await d.align_map('a', 'b', 0.5))
    with open(os.path.join(tmp, 'data', 'logs', 'align_map.log')) as f:
        check('对齐地图参数', "'a', 'b', '0.5'" in f.read())

    for raw, want in ((json.dumps({'ok': False, 'error': 'group busy'}).encode() + b'\n', 'group busy'),
                      (b'not json\n', 'invalid JSON'), (b'{"ok": true}', 'incomplete')):
        reply = raw
        try:
            await d.status(); check(f'supervisor 出错：{want}', False)
        except RuntimeError as e:
            check(f'supervisor 出错：{want}', want in str(e), e)
    reply = None

    await d.close()
    server.close(); await server.wait_closed()
    print(f'\n{"全部通过" if not fails else "失败：" + str(fails)}')
    sys.exit(1 if fails else 0)

asyncio.run(main())
