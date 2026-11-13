#!/usr/bin/env python3
"""DockerProcs 测试：起一个假的 Docker socket，真实执行它生成的 shell 脚本。

本地模拟器用 FakeProcs，绕过了这些脚本；这里验证 API 调用格式、pgrep 位掩码、pkill 流程和超时。
pgrep / pkill 的目标是本测试自己起的假进程，脚本里写 /go2w 的命令不测。
运行：python web/dev/test_procs.py（需要 aiohttp）
"""
import asyncio, os, subprocess, sys, tempfile, uuid
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'server'))
from aiohttp import web
import procs as P

execs, calls = {}, []

async def create(req):
    body = await req.json(); calls.append((req.match_info['c'], body))
    eid = uuid.uuid4().hex; execs[eid] = {'cmd': body['Cmd'], 'p': None}
    return web.json_response({'Id': eid}, status=201)

async def start(req):
    e = execs[req.match_info['id']]; body = await req.json(); assert body == {'Detach': True, 'Tty': False}
    e['p'] = subprocess.Popen(e['cmd'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return web.Response()

async def inspect(req):
    p = execs[req.match_info['id']]['p']; rc = p.poll()
    return web.json_response({'Running': rc is None, 'ExitCode': rc})

def fake(argv0_args):
    # 假进程：命令行看起来像 launch，收到 SIGINT 才退出
    return subprocess.Popen(['python3', '-c', 'import time\nwhile True: time.sleep(0.1)', *argv0_args])

async def main():
    sock = os.path.join(tempfile.mkdtemp(), 'd.sock')
    app = web.Application()
    app.router.add_post('/v1.41/containers/{c}/exec', create)
    app.router.add_post('/v1.41/exec/{id}/start', start)
    app.router.add_get('/v1.41/exec/{id}/json', inspect)
    r = web.AppRunner(app); await r.setup(); await web.UnixSite(r, sock).start()
    d, fails = P.DockerProcs(sock=sock), []
    def check(name, cond, extra=''):
        print(('PASS' if cond else 'FAIL'), name, extra); fails.append(name) if not cond else None

    st = await d.status(); check('没有进程时全部 false', not any(st.values()), st)
    check('请求发给 go2w_humble', calls[-1][0] == 'go2w_humble')
    check('状态检查不加载 ROS 环境', 'ros_env' not in calls[-1][1]['Cmd'][2])
    nav = fake(['ros2', 'launch', 'go2w_bringup', 'navigation.launch.py']); pat = fake(['ros2', 'run', 'go2w_bringup', 'patrol.py'])
    await asyncio.sleep(0.3)
    st = await d.status(); check('识别导航和巡逻', st['navigation'] and st['patrol'] and not st['mapping'] and not st['localization'] and not st['bag'], st)
    bag = fake(['ros2', 'bag', 'record', '/odom']); await asyncio.sleep(0.2)
    st = await d.status(); check('识别录包', st['bag'], st)
    await d.stop_bag(); await asyncio.sleep(0.2); check('stop_bag 停掉录包', bag.poll() is not None, bag.poll())
    check('stop_bag 不影响导航', nav.poll() is None)
    seg = fake(['/go2w/scripts/segment_recorder.py', '--prefix', 'go2w', '/odom']); await asyncio.sleep(0.2)
    st = await d.status(); check('识别分段录包', st['bag'], st)
    await d.stop_bag(); await asyncio.sleep(0.2); check('stop_bag 停掉分段录包', seg.poll() is not None, seg.poll())
    await d.stop_patrol(); await asyncio.sleep(0.2); check('stop_patrol 停掉巡逻', pat.poll() is not None)
    check('stop_patrol 不影响导航', nav.poll() is None)
    ok = await d.stop_modes(); await asyncio.sleep(0.2); check('stop_modes 停掉 launch 并报告成功', ok and nav.poll() is not None, (ok, nav.poll()))

    # 不响应 SIGINT 的 launch：要如实报告没停掉，不能假装成功
    stub = subprocess.Popen(['python3', '-c', 'import signal,time\nsignal.signal(signal.SIGINT, signal.SIG_IGN)\nwhile True: time.sleep(0.1)',
                             'ros2', 'launch', 'go2w_bringup', 'mapping.launch.py'])
    await asyncio.sleep(0.3)
    ok = await d.stop_modes(); check('launch 不退出时 stop_modes 报告失败', ok is False, ok); stub.kill(); stub.wait()

    for bad, fn in (('a;b', lambda n: d.start_bag(n, ['/odom'])), ('x y', d.save_map), ('$(id)', lambda n: d.start_mode('navigation', n))):
        try:
            await fn(bad); check(f'拒绝 {bad}', False)
        except AssertionError:
            check(f'拒绝 {bad}', True)
    # * = 录所有 topic：转成 --all，不能把 * 原样交给 shell（会被展开成文件名）
    for topics, want in ((['*'], ' --all '), (['*', '/odom'], ' --all /odom '), (['/odom', '/scan'], ' /odom /scan ')):
        await d.start_bag('t', topics); cmd = calls[-1][1]['Cmd'][2]
        check(f'start_bag {topics}', 'segment_recorder.py' in cmd and want in cmd and '*' not in cmd, cmd[-120:])
    for bad in (['**'], ['*;id'], []):
        try:
            await d.start_bag('t', bad); check(f'拒绝 topic {bad}', False)
        except AssertionError:
            check(f'拒绝 topic {bad}', True)
    await d.start_rollback(20); cmd = calls[-1][1]['Cmd'][2]
    check('回溯录制录所有 topic', 'rollback_recorder.py --seconds 20.0 --all >' in cmd, cmd[-100:])
    await asyncio.sleep(0.3)
    n0 = len(calls); await d.start_mode('navigation', 'lab'); cmd = calls[-1][1]['Cmd'][2]
    await asyncio.sleep(0.2)
    check('启动命令和 nav.sh 一致', 'ros2 launch go2w_bringup navigation.launch.py map:=/go2w/data/maps/lab.yaml' in cmd and 'ros_env' in cmd, cmd[-110:])
    check('一次操作一个 exec', len(calls) == n0 + 1)
    for e in execs.values():
        if e['p'] and e['p'].poll() is None: e['p'].kill()
    await d._session.close(); await r.cleanup()
    print(f'\n{"全部通过" if not fails else "失败：" + str(fails)}')

asyncio.run(main())
