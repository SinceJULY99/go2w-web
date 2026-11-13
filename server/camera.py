"""前置摄像头：收 Go2W 主控在内网组播的 RTP/H.264，原样转发给一个网页。

11-06 在扩展坞 eth0 上实测：主控 192.168.123.161 → 230.1.1.1:1720，H.264 High@4.0、1280×720、约 14 帧/秒、
约 1.45 Mbit/s，约每 2 s 一个关键帧（前面带 SPS/PPS）。
扩展坞不解码：只把 RTP 拼回完整的一帧（Annex-B），经 /ws 发出去，浏览器用 MSE 播放。
最多同时发给 MAX_VIEWERS 个网页（每多一个人，扩展坞 Wi-Fi 多发一路约 1.5 Mbit/s），每个网页单独排队、单独丢帧；
没人看时退出组播，不收任何数据。
本地调试：GO2W_CAM_ADDR=127.0.0.1:17200（单播），由 web/dev/fake_camera.py 回放录下的样本。
"""
import asyncio
import json
import os
import socket
import struct
import time

ADDR = os.environ.get('GO2W_CAM_ADDR', '230.1.1.1:1720')
IFACE = os.environ.get('GO2W_CAM_IF') or os.environ.get('GO2W_WIRED_IF', 'eth0')
NO_SIGNAL = 3.0      # 秒：这么久没收到包就提示「没有画面」
MAX_VIEWERS = int(os.environ.get('GO2W_CAM_MAX', '3'))
QUEUE_MAX = 30       # 约 2 s 的帧；网络跟不上时丢到下一个关键帧，不让延迟越积越大
START = b'\x00\x00\x00\x01'


class Depay:
    """RTP（RFC 6184：单个 NAL、STAP-A、FU-A）→ 一帧一帧的 Annex-B 数据。"""

    def __init__(self):
        self.nals, self.fu, self.ts, self.seq = [], None, None, None
        self.lost = False   # 当前帧缺包：整帧丢掉，并要求从下一个关键帧重新开始

    def push(self, d):
        """收一个 RTP 包，返回凑齐的帧列表 [(数据, 是否关键帧, RTP 时间戳, 之前是否丢过包)]。"""
        out = []
        if len(d) < 12 or d[0] >> 6 != 2:
            return out
        cc, ext, pad, marker = d[0] & 15, d[0] & 16, d[0] & 32, d[1] & 128
        seq, ts = struct.unpack('>HI', d[2:8])
        h = 12 + 4 * cc
        if ext:
            if len(d) < h + 4:
                return out
            h += 4 + 4 * struct.unpack('>H', d[h + 2:h + 4])[0]
        end = len(d) - (d[-1] if pad else 0)
        if self.seq is not None and seq != (self.seq + 1) & 0xFFFF:
            self.lost, self.fu = True, None
        self.seq = seq
        if self.ts is not None and ts != self.ts and (self.nals or self.fu):
            out += self.flush()          # 上一帧没带 marker 也要结束
        self.ts = ts
        if h >= end:
            return out
        p, t = d[h:end], d[h] & 31
        if 1 <= t <= 23:
            self.nals.append(bytes(p))
        elif t == 24:                     # STAP-A：一个包里几个小 NAL（常见于 SPS + PPS）
            i = 1
            while i + 2 <= len(p):
                n = struct.unpack('>H', p[i:i + 2])[0]
                if n and i + 2 + n <= len(p):
                    self.nals.append(bytes(p[i + 2:i + 2 + n]))
                i += 2 + n
        elif t == 28 and len(p) > 2:      # FU-A：一个大 NAL 拆成几个包
            if p[1] & 128:
                self.fu = bytearray([(p[0] & 0xE0) | (p[1] & 31)]) + p[2:]
            elif self.fu is not None:
                self.fu += p[2:]
            else:
                self.lost = True          # 中间的分片，开头丢了
            if p[1] & 64 and self.fu is not None:
                self.nals.append(bytes(self.fu))
                self.fu = None
        if marker:
            out += self.flush()
        return out

    def flush(self):
        nals, lost = self.nals, self.lost
        self.nals, self.fu, self.lost = [], None, False
        if not nals:
            return []
        key = any(n[0] & 31 == 5 for n in nals)
        return [(START + START.join(nals), key, self.ts, lost)]


class _Proto(asyncio.DatagramProtocol):
    def __init__(self, cam):
        self.cam = cam

    def datagram_received(self, data, addr):
        self.cam.on_packet(data)


class Viewer:
    """一个正在看画面的网页：自己的发送队列，网速跟不上时只丢它自己的帧。"""

    def __init__(self, ws, who):
        self.ws, self.who = ws, who
        self.queue = asyncio.Queue(QUEUE_MAX)
        self.need_key = True          # 刚加入 / 刚丢过帧：等下一个关键帧
        self.dropped = 0
        self.sender = None

    def start(self):
        self.sender = asyncio.create_task(self._send_loop())

    async def stop(self):
        if self.sender is not None:
            self.sender.cancel()
            await asyncio.gather(self.sender, return_exceptions=True)
            self.sender = None

    async def _send_loop(self):
        while True:
            blob = await self.queue.get()
            try:
                await self.ws.send_bytes(blob)
            except Exception:
                return

    def offer(self, key, blob):
        """收到一帧：返回是否放进了队列。"""
        if self.need_key and not key:
            return False
        if self.queue.full():
            # 网络跟不上：清空，等下一个关键帧，画面跳一下但延迟不会越积越多
            while not self.queue.empty():
                self.queue.get_nowait()
            self.dropped += 1
            self.need_key = True
            return False
        self.need_key = False
        self.queue.put_nowait(blob)
        return True


class Camera:
    def __init__(self, pack, kind, log):
        self.pack, self.kind, self.log = pack, kind, log
        self.viewers = {}             # ws → Viewer，最多 MAX_VIEWERS 个
        self.transport = None
        self.depay = Depay()
        self.last_ts = None
        self.last_rx = 0.0
        self.n_bytes = self.n_frames = 0
        self.size = None              # 从 SPS 解析出的分辨率

    # ---------- 组播 ----------
    def _socket(self):
        host, port = ADDR.rsplit(':', 1)
        port = int(port)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        if socket.inet_aton(host)[0] & 0xF0 == 0xE0:     # 224.0.0.0/4：组播，在有线口上加入
            s.bind(('', port))
            mreq = struct.pack('4s4si', socket.inet_aton(host), socket.inet_aton('0.0.0.0'), socket.if_nametoindex(IFACE))
            s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        else:
            s.bind((host, port))
        s.setblocking(False)
        return s

    async def _open(self):
        if self.transport is None:
            sock = self._socket()
            self.transport, _ = await asyncio.get_running_loop().create_datagram_endpoint(lambda: _Proto(self), sock=sock)
            self.depay, self.last_rx, self.last_ts = Depay(), 0.0, None
            multicast = socket.inet_aton(ADDR.rsplit(':', 1)[0])[0] & 0xF0 == 0xE0
            self.log('INFO', f"摄像头：开始接收 {ADDR}{f'（{IFACE}）' if multicast else ''}")

    def _close(self):
        if self.transport is not None:
            self.transport.close()   # 关闭 socket 时内核自动退出组播
            self.transport = None
            self.log('INFO', '摄像头：停止接收')

    # ---------- 观看者 ----------
    async def request(self, ws, who, on):
        """网页要看 / 不看了。返回给这个网页的状态消息。"""
        if not on:
            await self.release(ws)
            return {'state': 'off'}
        for w in [w for w in self.viewers if w.closed and w is not ws]:
            await self.release(w)
        v = self.viewers.get(ws)
        if v is not None:
            # 已经在看（比如浏览器播放器重建了）：清空，从下一个关键帧重新开始
            while not v.queue.empty():
                v.queue.get_nowait()
            v.need_key = True
            return {'state': 'starting', 'viewers': len(self.viewers), 'max': MAX_VIEWERS}
        if len(self.viewers) >= MAX_VIEWERS:
            return {'state': 'full', 'max': MAX_VIEWERS, 'who': [x.who for x in self.viewers.values()]}
        v = self.viewers[ws] = Viewer(ws, who)
        v.start()
        try:
            await self._open()
        except OSError as e:
            await self.release(ws)
            return {'state': 'error', 'msg': f'无法接收摄像头组播：{e}'}
        self.log('INFO', f'摄像头：{who} 开始观看（{len(self.viewers)} / {MAX_VIEWERS}）')
        return {'state': 'starting', 'viewers': len(self.viewers), 'max': MAX_VIEWERS}

    async def release(self, ws=None):
        """ws 不看了；不给 ws 就全部停掉。最后一个人走了就退出组播。"""
        for w in list(self.viewers) if ws is None else [ws]:
            v = self.viewers.pop(w, None)
            if v is not None:
                await v.stop()
                if ws is not None:
                    self.log('INFO', f'摄像头：{v.who} 停止观看（{len(self.viewers)} / {MAX_VIEWERS}）')
        if not self.viewers:
            self._close()

    @staticmethod
    async def _send_json(ws, m):
        try:
            await ws.send_str(json.dumps(m))
        except Exception:
            pass

    # ---------- 收包 ----------
    def on_packet(self, data):
        self.last_rx = time.monotonic()
        for au, key, ts, lost in self.depay.push(data):
            if lost:
                for v in self.viewers.values():
                    v.need_key = True
            if key and self.size is None:
                self.size = sps_size(au)
            dur = None
            if self.last_ts is not None:
                ms = ((ts - self.last_ts) & 0xFFFFFFFF) / 90.0
                dur = round(ms, 1) if 0 < ms < 1000 else None
            self.last_ts = ts
            if lost and not key:
                continue
            self.n_frames += 1
            self.n_bytes += len(au)
            blob = None
            for v in self.viewers.values():
                if v.need_key and not key:
                    continue
                if blob is None:
                    blob = self.pack(self.kind, {'key': key, 'dur': dur}, au)   # 打包一次，几个人共用
                v.offer(key, blob)

    async def monitor(self):
        """每秒给每个观看者发一次状态：帧率、码率、有没有信号、几个人在看。"""
        while True:
            await asyncio.sleep(1.0)
            for w in [w for w in self.viewers if w.closed]:
                await self.release(w)
            if not self.viewers:
                self.n_bytes = self.n_frames = 0
                continue
            quiet = time.monotonic() - self.last_rx if self.last_rx else None
            state = 'nosignal' if quiet is None or quiet > NO_SIGNAL else 'on'
            base = {'t': 'cam', 'state': state, 'fps': self.n_frames, 'kbps': round(self.n_bytes * 8 / 1000),
                    'size': self.size, 'viewers': len(self.viewers), 'max': MAX_VIEWERS}
            for v in list(self.viewers.values()):
                await self._send_json(v.ws, {**base, 'dropped': v.dropped})
            self.n_bytes = self.n_frames = 0


def sps_size(au):
    """从关键帧里的 SPS 读分辨率（只给界面显示用，解析不了就返回 None）。"""
    try:
        for nal in au.split(START):
            if nal and nal[0] & 31 == 7:
                return _parse_sps(nal)
    except (IndexError, ValueError):
        pass
    return None


def _parse_sps(nal):
    rbsp, zeros = bytearray(), 0
    for b in nal[1:]:
        if zeros >= 2 and b == 3:
            zeros = 0
            continue
        rbsp.append(b)
        zeros = zeros + 1 if b == 0 else 0
    bits = ''.join(f'{b:08b}' for b in rbsp)
    pos = 0

    def u(n):
        nonlocal pos
        v = int(bits[pos:pos + n], 2)
        pos += n
        return v

    def ue():
        nonlocal pos
        z = 0
        while bits[pos] == '0':
            z += 1
            pos += 1
        pos += 1
        v = (1 << z) - 1 + (int(bits[pos:pos + z], 2) if z else 0)
        pos += z
        return v

    def se():
        k = ue()
        return (k + 1) // 2 if k % 2 else -(k // 2)

    profile = u(8)
    u(16)
    ue()
    if profile in (100, 110, 122, 244, 44, 83, 86, 118, 128, 138, 139, 134, 135):
        if ue() == 3:
            u(1)
        ue(); ue(); u(1)
        if u(1):
            return None       # 带缩放矩阵，少见，不解析
    ue()
    poc = ue()
    if poc == 0:
        ue()
    elif poc == 1:
        u(1); se(); se()
        for _ in range(ue()):
            se()
    ue(); u(1)
    w = (ue() + 1) * 16
    hm = ue() + 1
    frame_mbs_only = u(1)
    h = (2 - frame_mbs_only) * hm * 16
    if not frame_mbs_only:
        u(1)
    u(1)
    if u(1):
        left, right, top, bottom = ue(), ue(), ue(), ue()
        w -= 2 * (left + right)
        h -= 2 * (2 - frame_mbs_only) * (top + bottom)
    return [w, h]
