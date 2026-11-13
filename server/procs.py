"""Console process lifecycle through the single-container runtime supervisor.

Long-lived ROS processes belong to supervisor groups. The web process only owns
bounded map helpers, launched with validated argv and its inherited ROS setup.
"""
import ast
import asyncio
import json
import math
import os
from pathlib import Path
import re
import signal
import stat

SAFE_NAME = re.compile(r'^[A-Za-z0-9_\-]{1,48}$')
SAFE_TOPIC = re.compile(r'^/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*$')
SAFE_PREFIX = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]{0,29}$')
MODES = ('mapping', 'localization', 'navigation')
BAG_SEGMENT = 30
ALL_TOPICS = '*'


def _name(value):
    if not isinstance(value, str) or not SAFE_NAME.fullmatch(value):
        raise ValueError('名字只能包含 1–48 个字母、数字、下划线或连字符')
    return value


def _seconds(value):
    if isinstance(value, bool):
        raise ValueError('时长必须在 1–3600 秒之间')
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError('时长必须在 1–3600 秒之间') from error
    if not math.isfinite(value) or not 1 <= value <= 3600:
        raise ValueError('时长必须在 1–3600 秒之间')
    return str(value)


class RuntimeProcs:
    def __init__(self, sock=None, data_dir=None, tools_dir='/opt/go2w/console/tools'):
        self.sock = str(sock or os.environ.get('GO2W_SUPERVISOR_SOCKET', '/run/go2w/supervisor.sock'))
        self.data_dir = Path(data_dir or os.environ.get('GO2W_DATA', '/go2w/data'))
        self.tools_dir = Path(tools_dir)
        self._helpers = set()

    def available(self):
        try:
            return stat.S_ISSOCK(os.stat(self.sock).st_mode)
        except OSError:
            return False

    async def _request(self, operation, *, timeout=35.0, **arguments):
        async def exchange():
            reader, writer = await asyncio.open_unix_connection(self.sock, limit=1024 * 1024)
            try:
                writer.write(json.dumps(dict(operation=operation, **arguments)).encode()+b'\n')
                await writer.drain()
                raw = await reader.readline()
                if not raw.endswith(b'\n'):
                    raise RuntimeError('Runtime supervisor returned an incomplete response')
                try:
                    response = json.loads(raw)
                except (ValueError, UnicodeDecodeError) as error:
                    raise RuntimeError('Runtime supervisor returned invalid JSON') from error
                if not isinstance(response, dict) or response.get('ok') is not True:
                    message = response.get('error') if isinstance(response, dict) else None
                    raise RuntimeError(message or 'Runtime supervisor request failed')
                if 'result' not in response:
                    raise RuntimeError('Runtime supervisor response has no result')
                return response['result']
            finally:
                writer.close()
                await writer.wait_closed()
        return await asyncio.wait_for(exchange(), timeout)

    async def status(self):
        groups = await self._request('status', timeout=5.0)
        if not isinstance(groups, dict):
            raise RuntimeError('Runtime supervisor returned invalid process status')
        navigation = groups.get('navigation', {})
        running = bool(navigation.get('running'))
        mode = navigation.get('environment', {}).get('GO2W_NAVIGATION_MODE') or 'navigation'
        if running and mode not in MODES:
            raise RuntimeError('Runtime supervisor returned an unknown navigation mode')
        result = {name: running and mode == name for name in MODES}
        result.update({name: bool(groups.get(name, {}).get('running'))
                       for name in ('patrol', 'bag', 'rollback', 'bridge')})
        result['bag_seg'] = result['bag']
        return result

    async def _start(self, name, environment=None):
        return await self._request('start', name=name, environment=environment or {})

    async def _stop(self, name):
        result = await self._request('stop', name=name)
        return isinstance(result, dict) and not result.get('running', False)

    async def start_mode(self, mode, map_yaml=None):
        if mode not in MODES:
            raise ValueError('不支持的运行档位')
        environment = {'GO2W_NAVIGATION_MODE': mode}
        if mode != 'mapping':
            environment['GO2W_MAP'] = _name(map_yaml)+'.yaml'
        return await self._start('navigation', environment)

    async def stop_modes(self):
        # These are the only groups owned by mode switching. In particular,
        # control, the C++ DDS recorder, and this web process are independent.
        stopped = True
        for name in ('patrol', 'navigation', 'bridge'):
            stopped = await self._stop(name) and stopped
        return stopped

    async def start_bridge(self):
        # The manager atomically guards exclusivity with navigation's bridge.
        return await self._start('bridge')

    async def stop_bridge(self):
        return await self._stop('bridge')

    async def start_patrol(self, loops=1):
        if type(loops) is not int or not 1 <= loops <= 1000:
            raise ValueError('巡逻圈数必须是 1–1000 的整数')
        return await self._start('patrol', {'GO2W_PATROL_LOOPS': str(loops)})

    async def stop_patrol(self):
        return await self._stop('patrol')

    async def start_bag(self, prefix, topics, seconds=BAG_SEGMENT):
        prefix = _name(prefix)
        if (not SAFE_PREFIX.fullmatch(prefix) or not isinstance(topics, (list, tuple)) or not topics
                or len(topics) > 256
                or any(not isinstance(topic, str) or
                       (topic != ALL_TOPICS and not SAFE_TOPIC.fullmatch(topic)) for topic in topics)):
            raise ValueError('录包前缀或 topic 列表无效')
        encoded = json.dumps(list(topics), separators=(',', ':'))
        if len(encoded) > 4096:
            raise ValueError('录包 topic 列表过长')
        return await self._start('bag', {'GO2W_BAG_PREFIX': prefix,
            'GO2W_BAG_TOPICS': encoded, 'GO2W_BAG_SECONDS': _seconds(seconds)})

    async def stop_bag(self):
        return await self._stop('bag')

    async def start_rollback(self, seconds):
        # The runtime entrypoint supplies --all, preserving the deployed UI.
        return await self._start('rollback', {'GO2W_ROLLBACK_SECONDS': _seconds(seconds)})

    async def stop_rollback(self):
        return await self._stop('rollback')

    async def _interrupt_helper(self, process):
        if process.returncode is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        # Map helpers share this process group with their ROS CLI children.
        # Never force-kill ROS processes; retain ownership until they exit.
        await process.wait()

    async def _run_helper(self, argv, log_name, timeout):
        logs = self.data_dir/'logs'
        logs.mkdir(parents=True, exist_ok=True)
        environment = dict(os.environ, GO2W_DATA=str(self.data_dir))
        with (logs/log_name).open('ab', buffering=0) as output:
            process = await asyncio.create_subprocess_exec(*map(str, argv),
                stdin=asyncio.subprocess.DEVNULL, stdout=output, stderr=asyncio.subprocess.STDOUT,
                env=environment, start_new_session=True)
        self._helpers.add(process)
        try:
            return await asyncio.wait_for(process.wait(), timeout)
        finally:
            try:
                await asyncio.shield(self._interrupt_helper(process))
            finally:
                self._helpers.discard(process)

    async def save_map(self, name):
        name = _name(name)
        return await self._run_helper(['bash', self.tools_dir/'save_map.sh', name],
                                      'save_map.log', 40.0) == 0

    async def align_map(self, source, target, angle):
        source, target = _name(source), _name(target)
        if source == target:
            raise ValueError('源地图和目标地图必须使用不同名字')
        if isinstance(angle, bool):
            raise ValueError('旋转角必须在 -90° 到 90° 之间')
        try:
            angle = float(angle)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError('旋转角必须在 -90° 到 90° 之间') from error
        if not math.isfinite(angle) or abs(angle) > math.pi/2:
            raise ValueError('旋转角必须在 -90° 到 90° 之间')
        return await self._run_helper(['python3', self.tools_dir/'align_map.py',
            self.data_dir/'maps', source, target, str(angle)], 'align_map.log', 120.0) == 0

    async def close(self):
        # Closing the web adapter never stops supervisor-owned modules.
        await asyncio.gather(*(self._interrupt_helper(process) for process in tuple(self._helpers)))


# Data lists are read locally from the same persistent data mount.
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
            try:
                origin = ast.literal_eval(meta.get('origin', ''))
                ox, oy = float(origin[0]), float(origin[1])
            except (ValueError, SyntaxError, TypeError, IndexError):
                ox = oy = None
            out.append({'name': name, 'w': w, 'h': h, 'res': float(meta.get('resolution', 0)), 'ox': ox, 'oy': oy,
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
        if name.startswith('.') or not os.path.isdir(d):   # .rollback 是回溯录制的临时目录
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
