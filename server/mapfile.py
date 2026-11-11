"""直接读 data/maps 里的地图文件（map_server 的 yaml + pgm），导航没开时网页也能看地图、编辑路线。

换算规则和 nav2 map_server 的 trinary 模式一致（我们的地图都是 map_saver_cli 存的 trinary）：
  p = (255 - 像素) / 255（negate: 1 时 p = 像素 / 255）
  p > occupied_thresh → 100，p < free_thresh → 0，其余 → -1（未知）
  图片第 0 行是地图最上边（y 最大），OccupancyGrid 第 0 行在 origin 一侧，所以要上下翻转。
"""
import ast
import os

import numpy as np


def read_yaml(path):
    """map_server 的 yaml 只有一层 key: value，不依赖 PyYAML。"""
    meta = {}
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.split('#', 1)[0]
            if ':' in line:
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
    return meta


def read_pgm(path):
    """二进制 PGM（P5，8 位）→ h×w uint8。头里可能有注释。"""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i+1].isspace():
            i += 1
        if data[i:i+1] == b'#':
            while i < len(data) and data[i:i+1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j+1].isspace():
            j += 1
        if j == i:
            raise ValueError(f'{path}：PGM 头不完整')
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b'P5':
        raise ValueError(f'{path}：只支持二进制 PGM（P5）')
    w, h, maxv = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxv > 255:
        raise ValueError(f'{path}：只支持 8 位 PGM')
    i += 1   # 头后面恰好一个空白字符
    if len(data) - i < w * h:
        raise ValueError(f'{path}：像素数据不完整')
    return np.frombuffer(data, dtype=np.uint8, count=w * h, offset=i).reshape(h, w), maxv


def load(yaml_path):
    """返回 dict：grid（h×w int8，0 / 100 / -1，行 0 在 origin 一侧）、res、ox、oy、oyaw。"""
    meta = read_yaml(yaml_path)
    res = float(meta['resolution'])
    origin = ast.literal_eval(meta['origin'])
    ox, oy = float(origin[0]), float(origin[1])
    oyaw = float(origin[2]) if len(origin) > 2 else 0.0
    occ_t = float(meta.get('occupied_thresh', 0.65))
    free_t = float(meta.get('free_thresh', 0.25))
    negate = int(meta.get('negate', 0))
    image = meta.get('image') or os.path.basename(yaml_path)[:-5] + '.pgm'
    px, maxv = read_pgm(os.path.join(os.path.dirname(yaml_path), image))
    v = px.astype(np.float32) / maxv
    p = v if negate else 1.0 - v
    grid = np.full(px.shape, -1, dtype=np.int8)
    grid[p > occ_t] = 100
    grid[p < free_t] = 0
    return {'grid': np.ascontiguousarray(grid[::-1]), 'res': res, 'ox': ox, 'oy': oy, 'oyaw': oyaw}
