"""摄像头协议测试：先运行 web/dev/run.sh（带假摄像头），再运行
    ~/.cache/go2w_web_venv/bin/python web/dev/test_camera_ws.py [ws://127.0.0.1:8080/ws]
"""
import asyncio
import json
import struct
import sys

import aiohttp

URL = sys.argv[1] if len(sys.argv) > 1 else 'ws://127.0.0.1:8080/ws'
assert URL.startswith(('ws://127.0.0.1:', 'ws://localhost:')), '只对本地假后端运行'
T_CAM = 7


class Client:
    def __init__(self, session):
        self.session, self.frames, self.states, self.logs = session, [], [], []

    async def open(self):
        self.ws = await self.session.ws_connect(URL)
        self.task = asyncio.create_task(self.read())
        return self

    async def read(self):
        async for m in self.ws:
            if m.type == aiohttp.WSMsgType.BINARY:
                kind, n = struct.unpack('<BI', m.data[:5])
                if kind == T_CAM:
                    self.frames.append(json.loads(m.data[5:5 + n]) | {'len': len(m.data) - 5 - n})
            elif m.type == aiohttp.WSMsgType.TEXT:
                d = json.loads(m.data)
                if d.get('t') == 'cam':
                    self.states.append(d)
                elif d.get('t') == 'logs':
                    self.logs += [x[2] for x in d['items']]

    async def cam(self, on):
        await self.ws.send_str(json.dumps({'t': 'cam', 'on': on}))

    async def until(self, fn, what, timeout=6.0):
        end = asyncio.get_running_loop().time() + timeout
        while not fn():
            assert asyncio.get_running_loop().time() < end, f'超时：{what}'
            await asyncio.sleep(0.05)

    async def close(self):
        await self.ws.close()
        self.task.cancel()


async def main():
    async with aiohttp.ClientSession() as s:
        a, b = await Client(s).open(), await Client(s).open()
        await asyncio.sleep(0.5)
        assert not a.frames and not b.frames, '没人申请时不发画面'

        await a.cam(True)
        await a.until(lambda: a.states and a.states[0]['state'] == 'starting', 'A 开始')
        await a.until(lambda: len(a.frames) >= 20, 'A 收到画面')
        assert a.frames[0]['key'] is True, '第一帧必须是关键帧'
        await a.until(lambda: any(x['state'] == 'on' for x in a.states), 'A 收到状态', 3)
        st = [x for x in a.states if x['state'] == 'on'][-1]
        assert st['size'] == [1280, 720] and 10 <= st['fps'] <= 18, st
        print(f"PASS 申请后从关键帧开始收到画面：{st['size'][0]}×{st['size'][1]}，{st['fps']} 帧/秒，{st['kbps']} kbit/s")

        await b.cam(True)
        await b.until(lambda: len(b.frames) >= 10, 'B 也收到画面')
        assert b.frames[0]['key'] is True, 'B 第一帧必须是关键帧'
        c = await Client(s).open()
        await c.cam(True)
        await c.until(lambda: len(c.frames) >= 10, 'C 也收到画面')
        n = len(a.frames)
        await a.until(lambda: len(a.frames) >= n + 10, 'A 继续收到画面')
        await a.until(lambda: any(x.get('viewers') == 3 for x in a.states), 'A 看到 3 人在看', 3)
        print('PASS 三个网页同时观看，各自从关键帧开始')

        d = await Client(s).open()
        await d.cam(True)
        await d.until(lambda: d.states and d.states[-1]['state'] == 'full', 'D 被拒')
        assert d.states[-1]['max'] == 3 and len(d.states[-1]['who']) == 3, d.states[-1]
        await asyncio.sleep(1.0)
        assert not d.frames, '满员时不应收到画面'
        print('PASS 第 4 个网页提示人数已满')

        await c.close()
        await asyncio.sleep(0.3)
        await d.cam(True)
        await d.until(lambda: len(d.frames) >= 5, 'C 走后 D 可以看')
        await d.cam(False)
        await asyncio.sleep(0.3)
        print('PASS 有人关闭后空出名额')

        await a.cam(False)
        await b.cam(False)
        await asyncio.sleep(0.5)
        n = len(b.frames)
        await asyncio.sleep(1.5)
        assert len(b.frames) == n, '关掉后不应再收到画面'
        await b.until(lambda: any('停止接收' in x for x in b.logs), '后端停止接收', 3)
        print('PASS 关掉后不再传输，后端退出接收')

        await a.cam(True)
        await a.until(lambda: a.states[-1]['state'] in ('starting', 'on'), 'A 重新申请')
        await a.close(); await b.close(); await d.close()
        print('PASS 重新申请正常')


asyncio.run(main())
