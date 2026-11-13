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

    async def start_bridge(self):
        self.sim.procs['bridge'] = True

    async def stop_bridge(self):
        self.sim.procs['bridge'] = False
        self.sim.armed = False
        await asyncio.sleep(0.3)
        return True

    async def stop_patrol(self):
        self.sim.later(lambda: self.sim.patrol_stop('patrol.py 收到 SIGINT'))

    async def start_bag(self, name, topics):
        self.sim.later(lambda: self.sim.bag_start(name, topics))

    async def stop_bag(self):
        self.sim.later(self.sim.bag_stop)
        await asyncio.sleep(0.5)

    async def start_rollback(self, seconds):
        self.sim.procs['rollback'] = True
        self._rollback = asyncio.create_task(self._rollback_loop(seconds))

    async def stop_rollback(self):
        self.sim.procs['rollback'] = False
        self._rollback.cancel()

    async def _rollback_loop(self, seconds):
        """和 scripts/rollback_recorder.py 相同的文件约定，包内容是假的。"""
        bags = Path(os.environ['GO2W_DATA']) / 'bags'
        request = bags / '.rollback' / 'request'
        while True:
            await asyncio.sleep(0.2)
            if not request.exists():
                continue
            parts = request.read_text().split()
            request.unlink()
            name = parts[0]
            seconds = float(parts[1]) if len(parts) > 1 else seconds
            (bags / name).mkdir(parents=True)
            (bags / name / f'{name}_0.mcap').write_bytes(b'\0' * int(seconds * 50_000))
            (bags / name / 'metadata.yaml').write_text(
                f'rosbag2_bagfile_information:\n  version: 5\n  storage_identifier: mcap\n'
                f'  duration:\n    nanoseconds: {int(seconds * 1e9)}\n')

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
