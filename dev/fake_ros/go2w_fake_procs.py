"""本地调试用：代替 web/server/procs.py 的 DockerProcs，「进程」都在假模拟器里。

由 go2w_web.py 在 GO2W_FAKE_PROCS=1 时加载（web/dev/run.sh 会设置）。
"""
import asyncio
import os
from pathlib import Path
import sys


class FakeProcs:
    def __init__(self, node):
        self.node = node

    @property
    def sim(self):
        return self.node.fake_sim

    def available(self):
        return True

    async def status(self):
        while not hasattr(self.node, 'fake_sim'):   # 后端起来时模拟器可能还没建好
            await asyncio.sleep(0.05)
        return dict(self.sim.procs)

    async def start_mode(self, mode, map_yaml=None):
        self.sim.later(lambda: self.sim.start_mode(mode, map_yaml))

    async def stop_modes(self):
        self.sim.later(self.sim.stop_modes)
        await asyncio.sleep(1.0)   # 真机上 launch 正常退出也要一会儿
        return True

    async def start_patrol(self, loops):
        self.sim.later(lambda: self.sim.patrol_start(loops))

    async def stop_patrol(self):
        self.sim.later(lambda: self.sim.patrol_stop('patrol.py 收到 SIGINT'))

    async def start_bag(self, name, topics):
        self.sim.later(lambda: self.sim.bag_start(name, topics))

    async def stop_bag(self):
        self.sim.later(self.sim.bag_stop)
        await asyncio.sleep(0.5)

    async def save_map(self, name):
        await asyncio.sleep(1.0)
        return self.sim.save_map(name)

    async def align_map(self, source, target, angle):
        script = Path(__file__).resolve().parents[3] / 'scripts/calib/align_map.py'
        data = Path(os.environ['GO2W_DATA'])
        with (data / 'logs/align_map.log').open('w') as log:
            process = await asyncio.create_subprocess_exec(
                sys.executable, str(script), str(data / 'maps'), source, target, str(angle),
                stdout=log, stderr=log)
            return await process.wait() == 0
