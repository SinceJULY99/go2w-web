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

    async def cam(self, on, take=False):
        await self.ws.send_str(json.dumps({'t': 'cam', 'on': on, 'take': take}))

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
        await b.until(lambda: b.states and b.states[-1]['state'] == 'busy', 'B 被拒')
        assert b.states[-1]['by'] == '127.0.0.1'
        await asyncio.sleep(1.0)
        assert not b.frames, '被拒的网页不应收到画面'
        print('PASS 同一时间只给一个网页，别人申请时提示正在被谁查看')

        await b.cam(True, take=True)
        await a.until(lambda: any(x['state'] == 'taken' for x in a.states), 'A 被接管')
        await b.until(lambda: len(b.frames) >= 10, 'B 收到画面')
        assert b.frames[0]['key'] is True
        n = len(a.frames)
        await asyncio.sleep(1.0)
        assert len(a.frames) <= n + 2, '被接管后 A 不应继续收到画面'
        print('PASS 接管：原来的网页收到提示并停止，新网页从关键帧开始')

        await b.cam(False)
        await asyncio.sleep(0.5)
        n = len(b.frames)
        await asyncio.sleep(1.5)
        assert len(b.frames) == n, '关掉后不应再收到画面'
        await b.until(lambda: any('停止接收' in x for x in b.logs), '后端停止接收', 3)
        print('PASS 关掉后不再传输，后端退出接收')

        await a.cam(True)
        await a.until(lambda: a.states[-1]['state'] in ('starting', 'on'), 'A 重新申请')
        await a.close()
        await asyncio.sleep(0.5)
        await b.cam(True)
        await b.until(lambda: b.states[-1]['state'] == 'starting', '看画面的网页关闭后，别人可以申请')
        await b.close()
        print('PASS 看画面的网页关闭后自动释放')


asyncio.run(main())
