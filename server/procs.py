"""运行档位的进程管理：在 go2w_humble 容器里启停 launch、巡逻、录包、存图（和 scripts/nav.sh 相同的命令）。

go2w_web 容器通过挂载的 /var/run/docker.sock 调 Docker Engine API 做 docker exec。
所有 exec 都是后台执行：输出写进 /go2w/data/logs/，后端直接读文件；需要结果时轮询退出码。
传进 shell 的名字都经过 SAFE_NAME / SAFE_TOPIC 校验，网页不能拼出任意命令。
"""
import asyncio
import os
import re
import time

import aiohttp

SAFE_NAME = re.compile(r'^[A-Za-z0-9_\-]{1,48}$')
SAFE_TOPIC = re.compile(r'^/[A-Za-z0-9_/]{1,96}$')
MODES = ('mapping', 'localization', 'navigation')
API = 'http://docker/v1.41'
ENV = 'source /go2w/docker/ros_env.sh >/dev/null 2>&1; '
LOGS = '/go2w/data/logs'
# 各档位的 launch 进程（[x] 写法让 pgrep / pkill 不匹配到自己的命令行）
LAUNCH = {m: f'[r]os2 launch go2w_bringup {m}.launch.py' for m in MODES}
PATROL, BAG = '[p]atrol.py', '[r]os2 bag record'


class DockerProcs:
    def __init__(self, container='go2w_humble', sock='/var/run/docker.sock'):
        self.container, self.sock = container, sock
        self._session = None

    def available(self):
        return os.path.exists(self.sock)

    async def session(self):
        if self._session is None:
            self._session = aiohttp.ClientSession(connector=aiohttp.UnixConnector(path=self.sock))
        return self._session

    async def _exec(self, script, ros_env=True):
        # 只用 pgrep / pkill 的命令不加载 ROS 环境：source setup.bash 要占近一秒 CPU，状态检查每 2 s 一次
        s = await self.session()
        async with s.post(f'{API}/containers/{self.container}/exec',
                          json={'Cmd': ['bash', '-c', (ENV if ros_env else '') + script], 'AttachStdout': False, 'AttachStderr': False}) as r:
            if r.status != 201:
                raise RuntimeError(f'docker exec 失败：HTTP {r.status} {(await r.text())[:200]}')
            eid = (await r.json())['Id']
        async with s.post(f'{API}/exec/{eid}/start', json={'Detach': True, 'Tty': False}) as r:
            if r.status != 200:
                raise RuntimeError(f'docker exec 启动失败：HTTP {r.status}')
        return eid

    async def run(self, script, timeout=30.0, ros_env=False):
        """执行并等它结束，返回退出码。"""
        eid = await self._exec(script, ros_env)
        s, t0 = await self.session(), time.monotonic()
        while True:
            async with s.get(f'{API}/exec/{eid}/json') as r:
                info = await r.json()
            if not info.get('Running'):
                return info.get('ExitCode')
            if time.monotonic() - t0 > timeout:
                raise asyncio.TimeoutError()
            await asyncio.sleep(0.1)

    async def spawn(self, script):
        """后台启动，不等结束（相当于 docker exec -d）。"""
        await self._exec(script)

    # ---------- 高层操作 ----------
    async def status(self):
        """哪些进程在跑：一次 exec，用退出码的各个位表示。"""
        checks = [*(LAUNCH[m] for m in MODES), PATROL, BAG]
        script = 'r=0; ' + ' '.join(f'pgrep -f "{p}" >/dev/null && r=$((r|{1 << i}));' for i, p in enumerate(checks)) + ' exit $r'
        code = await self.run(script, 5.0)
        keys = [*MODES, 'patrol', 'bag']
        return {k: bool(code & (1 << i)) for i, k in enumerate(keys)}

    async def start_mode(self, mode, map_yaml=None):
        assert mode in MODES
        arg = ''
        if mode != 'mapping':
            assert map_yaml and SAFE_NAME.match(map_yaml)
            arg = f' map:=/go2w/data/maps/{map_yaml}.yaml'
        log = f'{LOGS}/{mode}.log'
        await self.spawn(f'mkdir -p {LOGS}; [ -f {log} ] && mv {log} {LOGS}/{mode}_$(date +%Y%m%d_%H%M%S).log; '
                         f'chown -R 1000:1000 /go2w/data; exec ros2 launch go2w_bringup {mode}.launch.py{arg} > {log} 2>&1')

    async def stop_modes(self):
        """和 nav.sh stop 一样：先 SIGINT 让 launch 正常退出，10 s 后强杀残留。返回是否全部停掉。"""
        code = await self.run(
            f'pkill -INT -f "{PATROL}"; pkill -INT -f "[r]os2 launch go2w_bringup"; '
            f'for i in $(seq 1 20); do pgrep -f "[r]os2 launch go2w_bringup" >/dev/null || break; sleep 0.5; done; '
            f'pkill -9 -f "[f]astlio_mapping|[l]ivox_ros_driver2_node|[g]o2w_cmd_bridge.py"; '
            f'pgrep -f "[r]os2 launch go2w_bringup" >/dev/null && exit 1; exit 0', 30.0)
        return code == 0

    async def start_patrol(self, loops):
        await self.spawn(f'mkdir -p {LOGS}; exec ros2 run go2w_bringup patrol.py --ros-args -p loops:={int(loops)} > {LOGS}/patrol.log 2>&1')

    async def stop_patrol(self):
        await self.run(f'pkill -INT -f "{PATROL}"; exit 0', 5.0)

    async def start_bag(self, name, topics):
        assert SAFE_NAME.match(name) and topics and all(SAFE_TOPIC.match(t) for t in topics)
        await self.spawn(f'mkdir -p /go2w/data/bags {LOGS}; chown -R 1000:1000 /go2w/data; '
                         f'exec ros2 bag record -s mcap -o /go2w/data/bags/{name} {" ".join(topics)} > {LOGS}/bag_{name}.log 2>&1')

    async def stop_bag(self):
        await self.run(f'pkill -INT -f "{BAG}"; for i in $(seq 1 20); do pgrep -f "{BAG}" >/dev/null || break; sleep 0.5; done; '
                       f'chown -R 1000:1000 /go2w/data/bags; exit 0', 15.0)

    async def save_map(self, name):
        assert SAFE_NAME.match(name)
        return await self.run(f'bash /go2w/scripts/save_map.sh {name} > {LOGS}/save_map.log 2>&1', 40.0) == 0  # 脚本自己加载 ROS 环境

    async def align_map(self, source, target, angle):
        assert SAFE_NAME.fullmatch(source) and SAFE_NAME.fullmatch(target)
        return await self.run(
            f'python3 /go2w/scripts/calib/align_map.py /go2w/data/maps '
            f'{source} {target} {angle!r} > {LOGS}/align_map.log 2>&1', 120.0) == 0


# ---------- 直接读数据目录（web 容器和导航容器挂的是同一个仓库） ----------
def tail(path, n=12):
    try:
        with open(path, errors='replace') as f:
            return ''.join(f.readlines()[-n:]).strip()
    except OSError:
        return ''


def list_maps(maps_dir):
    out = []
    try:
        names = sorted(os.listdir(maps_dir))
    except OSError:
        return out
    for fn in names:
        if not fn.endswith('.yaml'):
            continue
        name, meta = fn[:-5], {}
        try:
            with open(os.path.join(maps_dir, fn)) as f:
                for line in f:
                    if ':' in line:
                        k, v = line.split(':', 1)
                        meta[k.strip()] = v.strip()
            w = h = None
            with open(os.path.join(maps_dir, meta.get('image', name + '.pgm')), 'rb') as f:
                tok = f.read(64).split()
            if tok and tok[0] == b'P5':
                w, h = int(tok[1]), int(tok[2])
            out.append({'name': name, 'w': w, 'h': h, 'res': float(meta.get('resolution', 0)),
                        'mtime': os.path.getmtime(os.path.join(maps_dir, fn))})
        except (OSError, ValueError, IndexError):
            continue
    return sorted(out, key=lambda m: -m['mtime'])


def list_bags(bags_dir):
    out = []
    try:
        names = os.listdir(bags_dir)
    except OSError:
        return out
    for name in names:
        d = os.path.join(bags_dir, name)
        if not os.path.isdir(d):
            continue
        size, dur = 0, None
        for fn in os.listdir(d):
            try:
                size += os.path.getsize(os.path.join(d, fn))
            except OSError:
                pass
        m = re.search(r'duration:\s*\n\s*nanoseconds:\s*(\d+)', tail(os.path.join(d, 'metadata.yaml'), 400))
        if m:
            dur = int(m.group(1)) / 1e9
        out.append({'name': name, 'size': round(size / 1048576, 1), 'dur': dur, 'mtime': os.path.getmtime(d)})
    return sorted(out, key=lambda b: -b['mtime'])
