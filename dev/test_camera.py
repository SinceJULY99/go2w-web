"""摄像头拆帧测试：用真实录下的组播样本（dev/camera_sample.rtp，11-06 在扩展坞上录的 10 s）检查 RTP → H.264。

    python3 dev/test_camera.py

样本格式：每个包前面是 <dH（录制时刻 s、包长），后面是原始 RTP。
装了 ffprobe 的话，还会用它独立解码拼出来的 H.264，确认分辨率和帧数。
"""
import os
import shutil
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'server'))
import camera as C  # noqa: E402


def packets(path):
    with open(path, 'rb') as f:
        data = f.read()
    i = 0
    while i + 10 <= len(data):
        t, n = struct.unpack('<dH', data[i:i + 10])
        yield t, data[i + 10:i + 10 + n]
        i += 10 + n


def main():
    sample = os.path.join(HERE, 'camera_sample.rtp')
    pkts = list(packets(sample))
    assert len(pkts) > 1000, len(pkts)

    d = C.Depay()
    frames = [f for _, p in pkts for f in d.push(p)]
    keys = [f for f in frames if f[1]]
    assert 120 <= len(frames) <= 170, f'帧数 {len(frames)}'
    assert len(keys) >= 4, f'关键帧 {len(keys)}'
    assert not any(f[3] for f in frames), '完整样本不应判为丢包'
    assert C.sps_size(keys[0][0]) == [1280, 720], C.sps_size(keys[0][0])
    for au, key, _, _ in keys:
        types = [n[0] & 31 for n in au.split(C.START) if n]
        assert 7 in types and 8 in types and 5 in types, f'关键帧缺 SPS/PPS：{types}'
    print(f'PASS 拆帧：{len(pkts)} 个包 → {len(frames)} 帧，{len(keys)} 个关键帧，1280×720')

    # 丢一个包：这一帧整帧丢掉并标记，之后的帧仍能拼出来
    d2, drop = C.Depay(), len(pkts) // 2
    out = [f for i, (_, p) in enumerate(pkts) if i != drop for f in d2.push(p)]
    assert any(f[3] for f in out), '丢包后应标记'
    assert len(out) >= len(frames) - 2
    print('PASS 丢包检测')

    # Camera.on_packet：从第一个关键帧开始，队列满时丢到下一个关键帧
    cam = C.Camera(lambda k, m, b: (m, b), 7, lambda *a: None)
    import asyncio
    cam.queue = asyncio.Queue(C.QUEUE_MAX)
    for _, p in pkts:
        cam.on_packet(p)
        if cam.queue.qsize() > 10:
            while not cam.queue.empty():
                cam.queue.get_nowait()
    first = None
    cam2 = C.Camera(lambda k, m, b: (m, b), 7, lambda *a: None)
    cam2.queue = asyncio.Queue(10000)
    for _, p in pkts:
        cam2.on_packet(p)
    first = cam2.queue.get_nowait()
    assert first[0]['key'] is True, '第一帧必须是关键帧'
    durs = []
    while not cam2.queue.empty():
        durs.append(cam2.queue.get_nowait()[0]['dur'])
    durs = [x for x in durs if x]
    assert 55 < sum(durs) / len(durs) < 85, f'平均帧间隔 {sum(durs) / len(durs):.1f} ms'
    assert cam2.size == [1280, 720]
    print(f'PASS 从关键帧开始发送，平均帧间隔 {sum(durs) / len(durs):.1f} ms（约 {1000 * len(durs) / sum(durs):.1f} 帧/秒）')

    if shutil.which('ffprobe'):
        start = frames.index(keys[0])
        with tempfile.NamedTemporaryFile(suffix='.h264', delete=False) as f:
            for au, *_ in frames[start:]:
                f.write(au)
        r = subprocess.run(['ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0',
                            '-show_entries', 'stream=codec_name,profile,width,height,nb_read_frames',
                            '-of', 'default=nw=1', f.name], capture_output=True, text=True)
        os.unlink(f.name)
        info = dict(line.split('=', 1) for line in r.stdout.split())
        assert info.get('codec_name') == 'h264' and info.get('width') == '1280' and info.get('height') == '720', r.stdout + r.stderr
        assert int(info['nb_read_frames']) == len(frames) - start, (info, len(frames) - start)
        assert not r.stderr.strip(), r.stderr
        print(f"PASS ffprobe 独立解码：{info['profile']} {info['width']}×{info['height']}，{info['nb_read_frames']} 帧，无报错")
    else:
        print('SKIP ffprobe（没装）')


if __name__ == '__main__':
    main()
