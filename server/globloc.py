#!/usr/bin/env python3
"""2D 全局定位（自动定位）：几帧静止的 360° scan 在整张占据栅格上暴力匹配，不需要初始位姿。

只依赖 numpy（有 scipy 时用它算距离变换，快一些）。
流程：
  1. 地图 → 似然场 field = exp(-d²/2σ²)，d 为到最近障碍的距离
  2. 粗搜：候选位置取地图上离障碍 > 0.2 m 的空闲格（每 COARSE 格取一个），航向每 1.5° 一档；
     粗似然场先做 COARSE×COARSE 滑窗取最大 —— 粗格的分数是其覆盖区域内任何位置分数的上界
     （Cartographer 分支定界的同一个思路，这里只分两层）
  3. 粗搜前 K 个（位置间隔 ≥ 0.5 m）在细似然场上局部细搜
  4. 输出最佳位姿、分数、「次优」（与最佳相距 > 1 m 或 > 30° 的最好结果）的分数 → 判断有没有歧义

11-10 离线评估（lab_20261105，两段实录 bag，真值 = AMCL + odom 外推）：
  单帧静止 scan 122/122 定对，位置误差中位 6 cm；遮挡 180° / 8 个人 + 遮挡 90° 时偶尔定错，
  错的全部是次优/最优 ≥ 0.85；镜像 scan（地图上不存在的地方）分数 0.52–0.75、比值 0.85–1.00。
  所以接受条件：分数 ≥ MIN_SCORE 且 次优/最优 ≤ MAX_RATIO。扩展坞上一次约 5 s。

go2w_web.py 把 scan / 地图存成 npz，用子进程跑本文件（numpy 的大计算会占住 GIL，放在网页后端进程里会卡住心跳 / 安全检查）：
  python3 globloc.py in.npz      → stdout 一行 JSON
npz 字段：grid（h×w int8，OccupancyGrid 原始值，行 0 在 origin 一侧）、res、origin（x, y）、pts（N×2，base_footprint 下的点）
"""
import json
import math
import sys
import time

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt as _edt
except ImportError:  # 容器里没有 scipy：限定最大距离，用平移取最小
    _edt = None

MIN_SCORE = 0.75   # 最优分数下限（平均似然，0..1）
MAX_RATIO = 0.80   # 次优 / 最优 上限：越接近 1 越有歧义


def distance_field(occ, max_cells):
    """每格到最近障碍格的距离（格），截断在 max_cells。"""
    if _edt is not None:
        return np.minimum(_edt(~occ), max_cells).astype(np.float32)
    h, w = occ.shape
    big = float(max_cells) ** 2
    r = max_cells
    pad = np.pad(occ, r, mode='constant', constant_values=False)
    d2 = np.full((h, w), big, np.float32)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            dd = dx * dx + dy * dy
            if dd > r * r:
                continue
            sub = pad[r + dy:r + dy + h, r + dx:r + dx + w]
            d2[sub & (d2 > dd)] = dd
    return np.sqrt(d2)


def sliding_max(a, k):
    """out[y, x] = max(a[y:y+k, x:x+k])，越界补 0。"""
    h, w = a.shape
    p = np.pad(a, ((0, k), (0, k)), mode='constant')
    out = np.zeros_like(a)
    for dy in range(k):
        for dx in range(k):
            np.maximum(out, p[dy:dy + h, dx:dx + w], out=out)
    return out


def thin(p, max_pts=180, voxel=0.10):
    """体素去重（避免近处密集点主导分数），再均匀抽样到 max_pts。"""
    if len(p) == 0:
        return p
    key = np.floor(p / voxel).astype(np.int64)
    _, idx = np.unique(key[:, 0] * 1000003 + key[:, 1], return_index=True)
    p = p[np.sort(idx)]
    if len(p) > max_pts:
        p = p[np.linspace(0, len(p) - 1, max_pts).astype(int)]
    return p


class GlobalLocalizer:
    def __init__(self, grid, res, origin_xy, sigma=0.10, coarse_sigma=0.20, coarse_cells=4, robot_clearance=0.20):
        t0 = time.time()
        self.res = float(res)
        self.ox, self.oy = float(origin_xy[0]), float(origin_xy[1])
        self.h, self.w = grid.shape
        occ = grid >= 65
        free = (grid >= 0) & (grid < 20)
        dm = distance_field(occ, int(math.ceil(1.0 / self.res))) * self.res
        # 地图外 / 未知区域的点不加分
        self.fine = np.exp(-dm * dm / (2 * sigma * sigma)).astype(np.float32)
        coarse = np.exp(-dm * dm / (2 * coarse_sigma * coarse_sigma)).astype(np.float32)
        self.k = coarse_cells
        self.coarse = sliding_max(coarse, coarse_cells)
        # 候选位置：空闲且离障碍有一定距离（狗不可能贴着墙 / 在墙里）
        ok = free & (dm > robot_clearance)
        ys, xs = np.nonzero(ok[::coarse_cells, ::coarse_cells])
        self.cand = np.stack([ys * coarse_cells, xs * coarse_cells], 1)
        self.prep_s = time.time() - t0

    def _offsets(self, pts, yaws):
        c, s = np.cos(yaws)[:, None], np.sin(yaws)[:, None]
        x = c * pts[None, :, 0] - s * pts[None, :, 1]
        y = s * pts[None, :, 0] + c * pts[None, :, 1]
        return np.rint(y / self.res).astype(np.int32), np.rint(x / self.res).astype(np.int32)

    def _score(self, field, cy, cx, dy, dx):
        Y = cy[:, None] + dy[None, :]
        X = cx[:, None] + dx[None, :]
        inside = (Y >= 0) & (Y < self.h) & (X >= 0) & (X < self.w)
        v = field[np.clip(Y, 0, self.h - 1), np.clip(X, 0, self.w - 1)]
        return (v * inside).mean(1)

    def to_world(self, cy, cx):
        return self.ox + (cx + 0.5) * self.res, self.oy + (cy + 0.5) * self.res

    def refine(self, pts, cy, cx, yaw, xy_cells, yaw_halfspan, yaw_step):
        yaws = yaw + np.arange(-yaw_halfspan, yaw_halfspan + 1e-9, yaw_step)
        d = np.arange(-xy_cells, xy_cells + 1)
        gy, gx = np.meshgrid(cy + d, cx + d, indexing='ij')
        gy, gx = gy.ravel(), gx.ravel()
        DY, DX = self._offsets(pts, yaws)
        best = (-1.0, cy, cx, yaw)
        for i, a in enumerate(yaws):
            s = self._score(self.fine, gy, gx, DY[i], DX[i])
            j = int(np.argmax(s))
            if s[j] > best[0]:
                best = (float(s[j]), int(gy[j]), int(gx[j]), float(a))
        return best

    def localize(self, pts, top_k=40, yaw_step_deg=1.5):
        t0 = time.time()
        yaws = np.deg2rad(np.arange(-180, 180, yaw_step_deg))
        cand = self.cand
        DY, DX = self._offsets(pts, yaws)
        S = np.zeros((len(cand), len(yaws)), np.float32)
        for i in range(len(yaws)):
            S[:, i] = self._score(self.coarse, cand[:, 0], cand[:, 1], DY[i], DX[i])
        coarse_s = time.time() - t0
        bi = S.argmax(1)
        bs = S[np.arange(len(S)), bi]
        picks, min_sep = [], (0.5 / self.res) ** 2
        for j in np.argsort(-bs):
            y, x = cand[j]
            if all((y - py) ** 2 + (x - px) ** 2 >= min_sep for py, px, _ in picks):
                picks.append((int(y), int(x), float(yaws[bi[j]])))
                if len(picks) >= top_k:
                    break
        step, res = math.radians(yaw_step_deg), []
        for y, x, a in picks:
            c = self.k // 2
            r = self.refine(pts, y + c, x + c, a, self.k, step, math.radians(0.5))
            res.append(self.refine(pts, r[1], r[2], r[3], 1, math.radians(0.5), math.radians(0.1)))
        res.sort(key=lambda r: -r[0])
        s0, y0, x0, a0 = res[0]
        bx, by = self.to_world(y0, x0)
        second, second_pose = 0.0, None
        for s, y, x, a in res[1:]:
            wx, wy = self.to_world(y, x)
            da = abs((a - a0 + math.pi) % (2 * math.pi) - math.pi)
            if math.hypot(wx - bx, wy - by) > 1.0 or da > math.radians(30):
                second, second_pose = s, [round(wx, 3), round(wy, 3), round((a + math.pi) % (2 * math.pi) - math.pi, 4)]
                break
        ratio = second / s0 if s0 > 0 else 1.0
        return {'x': round(bx, 4), 'y': round(by, 4), 'th': round((a0 + math.pi) % (2 * math.pi) - math.pi, 4),
                'score': round(s0, 4), 'second': round(second, 4), 'ratio': round(ratio, 4), 'second_pose': second_pose,
                'ok': bool(s0 >= MIN_SCORE and ratio <= MAX_RATIO),
                'n_pts': int(len(pts)), 'n_cand': int(len(cand)),
                'prep_s': round(self.prep_s, 2), 'coarse_s': round(coarse_s, 2), 'total_s': round(self.prep_s + time.time() - t0, 2)}


def main(path):
    d = np.load(path)
    L = GlobalLocalizer(d['grid'], float(d['res']), d['origin'])
    pts = thin(np.asarray(d['pts'], np.float64))
    if len(pts) < 30:
        print(json.dumps({'error': f'有效激光点太少（{len(pts)}）'}))
        return
    print(json.dumps(L.localize(pts)))


if __name__ == '__main__':
    main(sys.argv[1])
