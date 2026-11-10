"""本地调试用的假摄像头：按原来的时间间隔，把 dev/camera_sample.rtp 循环发到 127.0.0.1:17200（单播）。

run.sh 会自动启动它，并让后端用 GO2W_CAM_ADDR=127.0.0.1:17200 接收。
每一圈的 RTP 序号和时间戳接着上一圈往下走，后端看到的就是一路连续的视频。
"""
import os
import socket
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = ('127.0.0.1', int(os.environ.get('GO2W_FAKE_CAM_PORT', '17200')))


def load(path):
    with open(path, 'rb') as f:
        data = f.read()
    out, i = [], 0
    while i + 10 <= len(data):
        t, n = struct.unpack('<dH', data[i:i + 10])
        out.append((t, bytearray(data[i + 10:i + 10 + n])))
        i += 10 + n
    return out


def main():
    pkts = load(os.path.join(HERE, 'camera_sample.rtp'))
    span = pkts[-1][0] + 1 / 15
    seq0, ts0 = struct.unpack('>HI', pkts[0][1][2:8])
    ts_span = (struct.unpack('>I', pkts[-1][1][4:8])[0] - ts0 + 6000) & 0xFFFFFFFF
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    lap, start = 0, time.monotonic()
    while True:
        for t, p in pkts:
            wait = start + lap * span + t - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            seq, ts = struct.unpack('>HI', p[2:8])
            q = bytearray(p)
            q[2:8] = struct.pack('>HI', (seq + lap * len(pkts)) & 0xFFFF, (ts + lap * ts_span) & 0xFFFFFFFF)
            s.sendto(q, TARGET)
        lap += 1


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
