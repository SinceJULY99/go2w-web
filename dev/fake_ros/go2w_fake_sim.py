"""本地调试用的假机器人：加载真实地图，模拟导航栈的各个进程、Nav2、cmd_bridge、patrol.py 和里程计。

由假 rclpy 的 spin() 调用，把消息直接交给 go2w_web 节点注册的订阅回调。
行为尽量贴近真机，方便在本地测网页的控制、安全和进程管理逻辑：
  - 运行档位：建图 / 定位 / 导航三选一，启动约 3 s 后节点才出现；没启动时没有 scan、odom、服务
  - cmd_bridge：启动时上锁；/go2w_cmd_bridge/arm 解锁 / 上锁；max_vx、max_wz、cmd_timeout 可运行时修改；
    /cmd_vel 超过 cmd_timeout 没有新指令就停（键盘遥控用）
  - Nav2：navigate_to_pose 在地图上规划（0.2 m 栅格 A*）并行驶，10 Hz 反馈剩余距离；
    底盘上锁就不动；10 s 走不到 0.3 m（和 nav2.yaml 的 progress_checker 一样）→ 恢复行为，3 次后放弃
  - patrol.py：等解锁、等 /patrol_pose 的两个点，然后 A↔B 往返，日志文案和真的一样
  - map_server：load_map 切换地图；/initialpose 直接把机器人放到那里（假定位没有误差）
  - 录包、存图写进 GO2W_DATA（web/dev/run.sh 准备的数据副本），不碰仓库里的 data/
环境变量：
  GO2W_DATA             数据目录（maps / bags / logs），默认仓库的 data/
  GO2W_FAKE_MAP         启动时的地图 yaml，默认 $GO2W_DATA/maps/lab_20261021.yaml
  GO2W_FAKE_STAMP_RATE  /odom 时间戳走速相对真实时间的倍数，默认 1.0（调速度诊断时可设 1.5）
"""
import heapq
import math
import os
import queue
import time
import uuid
from types import SimpleNamespace as N

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.environ.get('GO2W_DATA') or os.path.normpath(os.path.join(HERE, '..', '..', '..', 'data'))
SAFE = 0.30           # 规划时离障碍物至少这么远（米），约等于机器人半宽 + 余量
COARSE = 0.2          # 规划栅格（米）
PROGRESS_RADIUS, PROGRESS_TIME = 0.3, 10.0
MAX_RECOVERIES = 3
START_DELAY = 3.0     # 档位启动到节点出现的时间（真机上 FAST-LIO 初始化更久）
MODES = ('mapping', 'localization', 'navigation')

ACCEPTED, EXECUTING, SUCCEEDED, CANCELED, ABORTED = 1, 2, 4, 5, 6


def load_map(yaml_path):
    """读 map_server 的 yaml + pgm，返回 OccupancyGrid 风格数组（行 0 在 origin 一侧）。"""
    meta = {}
    with open(yaml_path) as f:
        for line in f:
            if ':' in line:
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
    res = float(meta['resolution'])
    ox, oy = [float(x) for x in meta['origin'].strip('[]').split(',')[:2]]
    occ_t, free_t = float(meta.get('occupied_thresh', 0.65)), float(meta.get('free_thresh', 0.25))
    negate = int(meta.get('negate', 0))
    img = os.path.join(os.path.dirname(yaml_path), meta['image'])
    with open(img, 'rb') as f:
        data = f.read()
    # P5 头：magic、宽、高、最大值，中间可能有注释
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i+1].isspace():
            i += 1
        if data[i:i+1] == b'#':
            while data[i:i+1] != b'\n':
                i += 1
            continue
        j = i
        while not data[j:j+1].isspace():
            j += 1
        tokens.append(data[i:j]); i = j
    assert tokens[0] == b'P5', f'{img}: 只支持二进制 PGM（P5）'
    w, h, maxv = int(tokens[1]), int(tokens[2]), int(tokens[3])
    px = np.frombuffer(data, dtype=np.uint8, count=w*h, offset=i+1).reshape(h, w).astype(np.float32) / maxv
    p = px if negate else 1.0 - px
    grid = np.full((h, w), -1, dtype=np.int8)
    grid[p > occ_t] = 100
    grid[p < free_t] = 0
    return np.flipud(grid).copy(), res, ox, oy


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Sim:
    def __init__(self, node):
        self.node = node
        self.stamp_rate = float(os.environ.get('GO2W_FAKE_STAMP_RATE', '1.0'))
        self.jobs = queue.Queue()   # 其他线程交给模拟器线程执行的回调（模拟异步回复）
        self.t0 = time.monotonic()
        self.load(os.environ.get('GO2W_FAKE_MAP') or os.path.join(DATA, 'maps', 'lab_20261021.yaml'))

        # 起点：离地图原点 (0, 0) 最近的空旷处（建图通常从这里开始）
        ys, xs = np.nonzero(self.safe)
        k = np.argmin((self.ox + (xs+.5)*self.res)**2 + (self.oy + (ys+.5)*self.res)**2)
        self.x, self.y = self.ox + (xs[k]+.5)*self.res, self.oy + (ys[k]+.5)*self.res
        self.th, self.v, self.w_ = 0.0, 0.0, 0.0

        # 进程：开机时导航已经在跑（和平时 nav.sh start 之后一样）
        self.procs = {'mapping': False, 'localization': False, 'navigation': True, 'patrol': False, 'bag': False, 'bridge': False}
        self.ready_at = {'navigation': 0.0}
        # 假 cmd_bridge
        self.armed = False
        self.params = {'max_vx': 1.0, 'max_wz': 1.2, 'cmd_timeout': 0.3}
        self.teleop = None            # (vx, vy, wz, 收到时间)：/cmd_vel 上不是 Nav2 发的指令
        # 假 Nav2：同一时间只有一个目标
        self.goal = None
        # 假 patrol.py、录包
        self.pat = None
        self.bag = None

    def load(self, yaml_path):
        self.grid, self.res, self.ox, self.oy = load_map(yaml_path)
        self.map_name = os.path.basename(yaml_path)[:-5]
        if not hasattr(self, 'yaml_param'):
            self.yaml_param = self.map_name   # map_server 的 yaml_filename：启动时的地图
        self.h, self.w = self.grid.shape
        safe = self.grid == 0          # 离障碍物 SAFE 以外的空闲区域（4 邻域腐蚀）
        for _ in range(max(1, int(SAFE / self.res))):
            s = safe.copy()
            s[1:, :] &= safe[:-1, :]; s[:-1, :] &= safe[1:, :]; s[:, 1:] &= safe[:, :-1]; s[:, :-1] &= safe[:, 1:]
            safe = s
        self.safe = safe
        b = max(1, round(COARSE / self.res))
        H, W = self.h // b, self.w // b
        self.cb, self.coarse = b, safe[:H*b, :W*b].reshape(H, b, W, b).all(axis=(1, 3))
        self.cost = self.build_costmap()

    # ---------- 进程（档位、巡逻、录包） ----------
    def ready(self, mode):
        return self.procs[mode] and time.monotonic() >= self.ready_at.get(mode, 0.0)

    def any_ready(self):
        return any(self.ready(m) for m in MODES)

    def bridge_up(self):
        """有速度桥：导航档自带的，或不开导航时的遥控速度桥。"""
        return self.ready('navigation') or self.procs['bridge']

    def loc_ready(self):
        return self.ready('localization') or self.ready('navigation')

    def node_names(self):
        names = []
        if self.any_ready():
            names += ['livox_lidar_publisher', 'fast_lio', 'planar_odom', 'pcl2scan_slam']
        if self.loc_ready():
            names += ['map_server', 'amcl']
        if self.ready('mapping'):
            names += ['slam_toolbox']
        if self.ready('navigation'):
            names += ['bt_navigator', 'controller_server', 'planner_server']
        if self.bridge_up():
            names += ['go2w_cmd_bridge']
        if self.procs['patrol']:
            names += ['go2w_patrol']
        if self.procs['bag']:
            names += ['rosbag2_recorder']
        return names

    def topic_alive(self, topic):
        if topic == '/rosout':
            return True
        if topic == '/map':
            return self.loc_ready() or self.ready('mapping')
        if topic in ('/amcl_pose', '/particle_cloud', '/initialpose'):
            return self.loc_ready()
        if topic in ('/cmd_vel', '/go2w_cmd_bridge/armed'):
            return self.bridge_up()
        if topic in ('/global_costmap/costmap', '/local_costmap/costmap', '/plan', '/behavior_tree_log'):
            return self.ready('navigation')
        if topic == '/patrol_pose':
            return False
        return self.any_ready()

    def subscribers(self, topic):
        if topic == '/patrol_pose':
            return 1 if self.pat and time.monotonic() >= self.pat['ready_at'] else 0
        return 0

    def start_mode(self, mode, map_yaml):
        if map_yaml:
            self.load(os.path.join(DATA, 'maps', f'{map_yaml}.yaml'))
            self.yaml_param = map_yaml
            self.deliver_map()
        self.procs[mode] = True
        self.ready_at[mode] = time.monotonic() + START_DELAY
        self.rosout(20, 'launch', f'[{mode}.launch.py] all processes started')

    def stop_modes(self):
        self.patrol_stop('patrol.py 收到 SIGINT')
        if self.goal and self.goal['status'] in (ACCEPTED, EXECUTING):
            self.finish(CANCELED)
        for m in MODES:
            self.procs[m] = False
        self.procs['bridge'] = False   # 真机上同一个 pkill 模式会顺带杀掉遥控速度桥
        self.armed = False
        self.v = self.w_ = 0.0
        self.rosout(20, 'launch', 'all processes have finished cleanly')

    def bag_start(self, name, topics):
        self.procs['bag'] = True
        self.bag = {'name': name, 't0': time.monotonic(), 'topics': topics}
        self.rosout(20, 'rosbag2_recorder', f"Recording to '{name}'. Subscribed to {len(topics)} topics")

    def bag_stop(self):
        if not self.bag:
            return
        dur = time.monotonic() - self.bag['t0']
        d = os.path.join(DATA, 'bags', self.bag['name'])
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, f"{self.bag['name']}_0.mcap"), 'wb') as f:
            f.write(b'\0' * int(dur * 50_000))   # 假数据：每秒约 50 KB
        with open(os.path.join(d, 'metadata.yaml'), 'w') as f:
            f.write(f'rosbag2_bagfile_information:\n  version: 5\n  storage_identifier: mcap\n'
                    f'  duration:\n    nanoseconds: {int(dur * 1e9)}\n')
        self.procs['bag'], self.bag = False, None
        self.rosout(20, 'rosbag2_recorder', 'Recording stopped')

    def save_map(self, name):
        if not self.ready('mapping'):
            return False
        out = os.path.join(DATA, 'maps', name)
        img = np.where(self.grid == 100, 0, np.where(self.grid == 0, 254, 205)).astype(np.uint8)
        with open(out + '.pgm', 'wb') as f:
            f.write(f'P5\n{self.w} {self.h}\n255\n'.encode() + np.flipud(img).tobytes())
        with open(out + '.yaml', 'w') as f:
            f.write(f'image: {name}.pgm\nresolution: {self.res}\norigin: [{self.ox}, {self.oy}, 0.0]\n'
                    f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        return True

    # ---------- 假 patrol.py（日志文案和 scripts/patrol.py 一致，网页靠它跟踪进度） ----------
    def plog(self, level, msg):
        self.rosout(level, 'go2w_patrol', msg)

    def patrol_start(self, loops):
        self.procs['patrol'] = True
        self.pat = {'loops': loops, 'points': [], 'legs': 0, 'i': 0, 'state': 'wait_armed', 'warned': False,
                    'ready_at': time.monotonic() + 1.5, 'gid': None, 'dwell_until': 0.0, 'leg_t0': 0.0}

    def patrol_stop(self, reason):
        p = self.pat
        if not p:
            return
        if p['gid'] and self.goal and self.goal['id'] == p['gid'] and self.goal['status'] in (ACCEPTED, EXECUTING):
            self.finish(CANCELED)
            self.plog(30, '已取消当前导航目标')
        if reason:
            self.plog(30, f'巡逻结束：{reason}')
        self.procs['patrol'], self.pat = False, None

    def patrol_step(self, now):
        p = self.pat
        if not p or now < p['ready_at']:
            return
        if p['state'] == 'wait_armed':
            if self.armed:
                self.plog(20, '在 RViz 里用 Patrol Pose 依次设 A、B（按下定位置、拖动定朝向）')
                p['state'] = 'wait_points'
            elif not p['warned']:
                p['warned'] = True
                self.plog(30, '速度桥还没解锁，等待解锁（另开终端执行 nav.sh arm）……')
            return
        if not self.armed:
            self.plog(40, '速度桥已上锁')
            return self.patrol_stop(None)
        if p['state'] == 'wait_points':
            if len(p['points']) == 2:
                self.plog(20, '开始巡逻：%s，每个点停 3.0 s' % ('一直往返' if p['loops'] == 0 else '往返 %d 次' % p['loops']))
                p['state'] = 'go'
        elif p['state'] == 'go':
            x, y, yaw = p['points'][p['i']]
            goal = N(pose=N(pose=N(position=N(x=x, y=y), orientation=N(z=math.sin(yaw/2), w=math.cos(yaw/2)))))
            p['gid'] = self.nav_send_goal(goal, None).goal_id.uuid
            p['gid'] = bytes(p['gid'])
            p['leg_t0'], p['state'] = now, 'moving'
            self.plog(20, '→ 去 %s 点' % 'AB'[p['i']])
        elif p['state'] == 'moving':
            g = self.goal
            if not g or g['id'] != p['gid'] or g['status'] in (ACCEPTED, EXECUTING):
                return
            if g['status'] != SUCCEEDED:
                self.plog(40, '去 %s 点失败（状态 %d）' % ('AB'[p['i']], g['status']))
                return self.patrol_stop('去 %s 点失败' % 'AB'[p['i']])
            p['legs'] += 1
            self.plog(20, '到达 %s 点，用时 %.1f s（第 %d 段）' % ('AB'[p['i']], now - p['leg_t0'], p['legs']))
            p['i'] = 1 - p['i']
            if p['loops'] and p['legs'] >= 2 * p['loops'] + 1:
                self.plog(20, '巡逻完成')
                return self.patrol_stop(None)
            p['dwell_until'], p['state'] = now + 3.0, 'dwell'
        elif p['state'] == 'dwell' and now >= p['dwell_until']:
            p['state'] = 'go'

    # ---------- 线程间 ----------
    def later(self, fn):
        self.jobs.put(fn)

    # ---------- 假服务 ----------
    def has_service(self, name):
        if name.startswith('/go2w_cmd_bridge/'):
            return self.bridge_up()
        if name == '/navigate_to_pose/_action/cancel_goal' or '/clear_except_' in name:
            return self.ready('navigation')
        if name.startswith('/map_server/'):
            return self.loc_ready()
        return False

    def call_service(self, name, req):
        from action_msgs.srv import CancelGoal
        from nav2_msgs.srv import LoadMap
        from rcl_interfaces.msg import ParameterType, ParameterValue, SetParametersResult
        from rcl_interfaces.srv import GetParameters, SetParameters
        from std_srvs.srv import SetBool
        if '/clear_except_' in name:
            from nav2_msgs.srv import ClearCostmapExceptRegion
            self.costmap_clears = getattr(self, 'costmap_clears', 0) + 1
            return ClearCostmapExceptRegion.Response()
        if name == '/map_server/load_map':
            path = req.map_url.replace('/go2w/data', DATA, 1)
            if not os.path.exists(path):
                return LoadMap.Response(1)
            self.load(path)
            self.deliver_map()
            return LoadMap.Response(0)
        if name == '/map_server/get_parameters':
            # 真机的 map_server：load_map 服务换图后，yaml_filename 参数仍是启动时的那张（11-06 实测）
            return GetParameters.Response([ParameterValue(4, string_value=f'/go2w/data/maps/{self.yaml_param}.yaml')])
        if name.endswith('/arm'):
            if req.data:
                self.armed = True
                return SetBool.Response(True, '已解锁')
            self.armed = False
            return SetBool.Response(True, '已上锁')
        if name.endswith('/get_parameters'):
            return GetParameters.Response([ParameterValue(ParameterType.PARAMETER_DOUBLE, self.params[n])
                                           for n in req.names if n in self.params])
        if name.endswith('/set_parameters'):
            out = []
            for p in req.parameters:
                if p.name in self.params and p.value.double_value >= 0:
                    self.params[p.name] = p.value.double_value
                    out.append(SetParametersResult(True))
                else:
                    out.append(SetParametersResult(False, f'{p.name} 必须是非负数'))
            return SetParameters.Response(out)
        if name.endswith('/cancel_goal'):
            if self.goal and self.goal['status'] in (ACCEPTED, EXECUTING):
                self.finish(CANCELED)
            return CancelGoal.Response()
        raise KeyError(name)

    def on_publish(self, topic, msg):
        if topic == '/initialpose' and self.loc_ready():
            p = msg.pose.pose
            self.later(lambda: self.teleport(p.position.x, p.position.y, 2*math.atan2(p.orientation.z, p.orientation.w)))
        elif topic == '/patrol_pose' and self.pat and len(self.pat['points']) < 2:
            p = msg.pose
            yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
            self.pat['points'].append((p.position.x, p.position.y, yaw))
            self.plog(20, '%s 点：(%.2f, %.2f)，朝向 %.0f°' % ('AB'[len(self.pat['points']) - 1], p.position.x, p.position.y, math.degrees(yaw)))
        elif topic == '/cmd_vel':
            self.teleop = (msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())

    def teleport(self, x, y, th):
        self.x, self.y, self.th, self.v, self.w_ = x, y, th, 0.0, 0.0

    # ---------- 假 Nav2 ----------
    def nav_ready(self):
        return self.ready('navigation')

    def nav_send_goal(self, goal, fb_cb):
        from rclpy._fake import Future
        if self.goal and self.goal['status'] in (ACCEPTED, EXECUTING):
            self.finish(ABORTED)     # 新目标抢占旧目标（Nav2 里旧目标显示为被抢占 / 放弃）
        gid = uuid.uuid4().bytes
        p = goal.pose.pose
        self.goal = {'id': gid, 'x': p.position.x, 'y': p.position.y, 'th': 2*math.atan2(p.orientation.z, p.orientation.w),
                     'status': ACCEPTED, 'fb': fb_cb, 'result': Future(), 'path': None, 'i': 0,
                     't0': time.monotonic(), 'plan_at': time.monotonic() + 0.3, 'recoveries': 0,
                     'mark': (time.monotonic(), self.x, self.y), 'recover_until': 0.0}
        self.publish_status()
        self.bt(('ComputePathToPose', 'IDLE', 'RUNNING'))
        def cancel():
            f = Future()

            def apply():
                if self.goal and self.goal['id'] == gid and self.goal['status'] in (ACCEPTED, EXECUTING):
                    self.finish(CANCELED)
                f.set_result(None)

            self.later(apply)
            return f

        return N(accepted=True, goal_id=N(uuid=list(gid)), get_result_async=lambda: self.goal_by(gid)['result'],
                 cancel_goal_async=cancel)

    def goal_by(self, gid):
        if self.goal and self.goal['id'] == gid:
            return self.goal
        from rclpy._fake import Future
        f = Future(); f.set_result(N(status=ABORTED, result=None))
        return {'result': f}

    def finish(self, status):
        g = self.goal
        g['status'] = status
        self.publish_status()
        self.bt(('FollowPath', 'RUNNING', 'IDLE'), ('ComputePathToPose', 'RUNNING', 'IDLE'), ('Wait', 'RUNNING', 'IDLE'))
        g['result'].set_result(N(status=status, result=None))

    def publish_status(self):
        from action_msgs.msg import GoalStatusArray
        g = self.goal
        self.node.deliver('/navigate_to_pose/_action/status', GoalStatusArray([(g['id'], g['status'])] if g else []))

    def bt(self, *events):
        from nav2_msgs.msg import BehaviorTreeLog
        self.node.deliver('/behavior_tree_log', BehaviorTreeLog(events))

    def compute_path(self, goal):
        from nav_msgs.msg import Path
        from rclpy._fake import Future
        p = goal.goal.pose.position
        a = goal.start.pose.position
        start = (a.x, a.y) if goal.use_start else (self.x, self.y)
        points = self.plan(p.x, p.y, start)
        path = Path([start, *points] if points is not None else [])
        result = Future()
        result.set_result(N(status=SUCCEEDED if path.poses else ABORTED, result=N(path=path)))
        return N(accepted=True, get_result_async=lambda: result)

    def plan(self, gx, gy, start=None):
        """0.2 m 栅格上的 A*，再用细栅格做视线捷径。返回航点列表或 None。"""
        b, C = self.cb, self.coarse
        H, W = C.shape

        def cell(x, y):
            return int((x - self.ox) / self.res) // b, int((y - self.oy) / self.res) // b

        def snap(c):  # 起点 / 终点贴着障碍物时，找 0.6 m 内最近的可通行格子
            cx, cy = c
            for r in range(0, 4):
                best = None
                for dx in range(-r, r+1):
                    for dy in range(-r, r+1):
                        x, y = cx+dx, cy+dy
                        if 0 <= x < W and 0 <= y < H and C[y, x] and (best is None or dx*dx + dy*dy < best[0]):
                            best = (dx*dx + dy*dy, (x, y))
                if best:
                    return best[1]
            return None

        start = start if start is not None else (self.x, self.y)
        s, t = snap(cell(*start)), snap(cell(gx, gy))
        if s is None or t is None:
            return None
        openq, came, g = [(0.0, s)], {s: None}, {s: 0.0}
        while openq:
            _, c = heapq.heappop(openq)
            if c == t:
                break
            for dx, dy, l in ((1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)):
                n = (c[0]+dx, c[1]+dy)
                if not (0 <= n[0] < W and 0 <= n[1] < H) or not C[n[1], n[0]]:
                    continue
                ng = g[c] + l
                if ng < g.get(n, 1e9):
                    g[n], came[n] = ng, c
                    heapq.heappush(openq, (ng + math.hypot(t[0]-n[0], t[1]-n[1]), n))
        if t not in came:
            return None
        cells, c = [], t
        while c is not None:
            cells.append(c); c = came[c]
        pts = [(self.ox + (cx+.5)*b*self.res, self.oy + (cy+.5)*b*self.res) for cx, cy in reversed(cells)]
        pts = [start] + pts[1:-1] + [(gx, gy)]
        out, i = [pts[0]], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self.clear(pts[i], pts[j]):
                j -= 1
            out.append(pts[j]); i = j
        return out[1:]

    def clear(self, a, b):
        n = max(2, int(math.hypot(b[0]-a[0], b[1]-a[1]) / self.res))
        for k in range(n+1):
            x, y = a[0] + (b[0]-a[0])*k/n, a[1] + (b[1]-a[1])*k/n
            gx, gy = int((x - self.ox) / self.res), int((y - self.oy) / self.res)
            if not (0 <= gx < self.w and 0 <= gy < self.h) or self.grid[gy, gx] != 0:
                return False
        return True

    def remaining(self):
        g = self.goal
        if not g['path']:
            return math.hypot(g['x']-self.x, g['y']-self.y)
        pts = [(self.x, self.y)] + g['path'][g['i']:]
        return sum(math.hypot(pts[k+1][0]-pts[k][0], pts[k+1][1]-pts[k][1]) for k in range(len(pts)-1))

    def nav_step(self, now):
        """返回 (目标线速度, 目标角速度)。"""
        g = self.goal
        if not g or g['status'] not in (ACCEPTED, EXECUTING):
            return 0.0, 0.0
        if not self.ready('navigation'):
            return 0.0, 0.0
        if g['status'] == ACCEPTED:
            if now < g['plan_at']:
                return 0.0, 0.0
            g['path'] = self.plan(g['x'], g['y'])
            if g['path'] is None:
                self.node.get_logger().warn('fake nav: 找不到路径，放弃目标')
                self.rosout(40, 'planner_server', 'GridBased: failed to create plan, no valid path found.')
                self.finish(ABORTED)
                return 0.0, 0.0
            g['status'] = EXECUTING
            self.publish_status()
            self.bt(('ComputePathToPose', 'RUNNING', 'SUCCESS'), ('FollowPath', 'IDLE', 'RUNNING'))
            self.node.deliver('/plan', self.path_msg())
            self.rosout(20, 'controller_server', 'Received a goal, begin computing control effort.')
            g['mark'] = (now, self.x, self.y)
        if now < g['recover_until']:
            return 0.0, 0.0
        # 进展检查：和 nav2.yaml 的 progress_checker 一样，10 s 内没走出 0.3 m 就恢复，多次后放弃
        t0, mx, my = g['mark']
        if math.hypot(self.x-mx, self.y-my) > PROGRESS_RADIUS:
            g['mark'] = (now, self.x, self.y)
        elif now - t0 > PROGRESS_TIME:
            g['recoveries'] += 1
            if g['recoveries'] > MAX_RECOVERIES:
                self.node.get_logger().warn('fake nav: 多次恢复仍没有进展，放弃目标')
                self.finish(ABORTED)
                return 0.0, 0.0
            self.bt(('FollowPath', 'RUNNING', 'FAILURE'), ('Wait', 'IDLE', 'RUNNING'))
            self.rosout(40, 'controller_server', 'Failed to make progress')
            self.rosout(30, 'bt_navigator', f"Running recovery: Wait ({g['recoveries']}/{MAX_RECOVERIES})")
            g['recover_until'] = now + 3.0
            g['mark'] = (now + 3.0, self.x, self.y)
            return 0.0, 0.0
        if g['recover_until']:               # 恢复行为结束，重新跟随路径
            g['recover_until'] = 0.0
            self.bt(('Wait', 'RUNNING', 'SUCCESS'), ('FollowPath', 'IDLE', 'RUNNING'))
        vmax, wmax = min(self.params['max_vx'], 0.6), self.params['max_wz']
        if g['i'] < len(g['path']):
            tx, ty = g['path'][g['i']]
            d = math.hypot(tx-self.x, ty-self.y)
            last = g['i'] == len(g['path']) - 1
            if d < (0.12 if last else 0.3):
                g['i'] += 1
                return self.v, 0.0
            err = wrap(math.atan2(ty-self.y, tx-self.x) - self.th)
            if abs(err) > 0.5:
                return 0.0, math.copysign(min(wmax*.7, abs(err)*1.5 + .1), err)   # RotationShim：先原地转向
            return min(vmax, 0.12 + (d if last else 1.0)*0.6), max(-wmax, min(wmax, 2.0*err))
        err = wrap(g['th'] - self.th)        # 到点后对准目标朝向
        if abs(err) < 0.05:
            self.finish(SUCCEEDED)
            return 0.0, 0.0
        return 0.0, math.copysign(min(wmax*.6, abs(err)*1.5 + .05), err)

    def feedback(self, now):
        g = self.goal
        if not g or g['status'] != EXECUTING:
            return
        rem = self.remaining()
        nav = now - g['t0']
        eta = rem / max(0.1, min(self.params['max_vx'], 0.6) * 0.8)
        dur = lambda s: N(sec=int(s), nanosec=int((s % 1) * 1e9))
        msg = N(goal_id=N(uuid=list(g['id'])),
                feedback=N(distance_remaining=rem, navigation_time=dur(nav), estimated_time_remaining=dur(eta),
                           number_of_recoveries=g['recoveries']))
        if g['fb']:
            g['fb'](msg)
        self.node.deliver('/navigate_to_pose/_action/feedback', msg)   # 所有目标都有，含 patrol 发的

    # ---------- 运动 ----------
    def step(self, dt, now):
        tv, tw = self.nav_step(now)
        nav_moving = self.goal and self.goal['status'] in (ACCEPTED, EXECUTING)
        t = self.teleop
        if t and not nav_moving and now - t[3] < self.params['cmd_timeout']:
            tv, tw = t[0], t[2]          # 键盘遥控：cmd_bridge 只认最新的 /cmd_vel
        if not self.armed or not self.bridge_up():
            tv = tw = 0.0          # 上锁：cmd_bridge 不执行任何速度指令
        self.v += max(-dt, min(dt, tv - self.v))            # 加速度 1 m/s²
        self.w_ += max(-2*dt, min(2*dt, tw - self.w_))
        if not self.armed:
            self.v *= max(0.0, 1 - 6*dt); self.w_ *= max(0.0, 1 - 6*dt)
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        self.th = wrap(self.th + self.w_*dt)

    # ---------- 传感器与各种消息 ----------
    def scan(self, n=360, rmax=15.0):
        """base_footprint 坐标系下、从 -π 起均匀分布的射线距离（和 pcl2scan 输出一致）。"""
        a = self.th + np.linspace(-math.pi, math.pi, n, endpoint=False)
        r = np.arange(0.1, rmax, self.res)
        X = self.x + np.cos(a)[:, None] * r[None, :]
        Y = self.y + np.sin(a)[:, None] * r[None, :]
        gx, gy = np.floor((X - self.ox) / self.res).astype(int), np.floor((Y - self.oy) / self.res).astype(int)
        inside = (gx >= 0) & (gy >= 0) & (gx < self.w) & (gy < self.h)
        hit = np.zeros_like(inside)
        hit[inside] = self.grid[gy[inside], gx[inside]] == 100
        first = np.where(hit.any(1), hit.argmax(1), -1)
        ranges = np.where(first >= 0, r[np.maximum(first, 0)], np.inf)
        return (ranges + np.random.normal(0, 0.01, n) * np.isfinite(ranges)).astype(np.float32)

    def stamp(self):
        t = (time.monotonic() - self.t0) * self.stamp_rate
        return int(t), int((t % 1) * 1e9)

    def tilt(self):
        """机身 (roll, pitch)，弧度：静止约 -0.4° / 0.9°，加速抬头、转弯侧倾，外加一点抖动。"""
        t = time.monotonic()
        return (math.radians(-0.4 + 0.6*self.w_ + 0.06*math.sin(t*5.3)),
                math.radians(0.9 + 1.5*self.v + 0.08*math.sin(t*7)))

    def build_costmap(self):
        """和 nav2.yaml 一样的膨胀：占用 100，内切半径内 99，0.7 m 内按 exp(-2.5·(d-0.24)) 递减，未知 -1。"""
        occ = self.grid == 100
        dist = np.full(self.grid.shape, 10**4, dtype=np.int32)
        dist[occ] = 0
        cur = occ
        for d in range(1, int(0.7 / self.res) + 1):
            n = cur.copy()
            n[1:, :] |= cur[:-1, :]; n[:-1, :] |= cur[1:, :]; n[:, 1:] |= cur[:, :-1]; n[:, :-1] |= cur[:, 1:]
            if d % 2 == 0:   # 4 邻域和 8 邻域交替膨胀：近似圆形，不然膨胀区是菱形
                n[1:, 1:] |= cur[:-1, :-1]; n[:-1, :-1] |= cur[1:, 1:]; n[1:, :-1] |= cur[:-1, 1:]; n[:-1, 1:] |= cur[1:, :-1]
            dist[n & (dist > d)] = d
            cur = n
        dm = dist * self.res
        cost = np.where(dm <= 0.24, 99, np.where(dm <= 0.7, np.maximum(1, np.round(98 * np.exp(-2.5 * (dm - 0.24)))), 0))
        cost[occ] = 100
        cost[(self.grid == -1) & ~occ] = -1
        return cost.astype(np.int8)

    def costmap_msg(self, local):
        from nav_msgs.msg import OccupancyGrid
        m = OccupancyGrid()
        m.info.resolution = self.res
        if local:   # 6 × 6 m 滚动窗口，对齐地图栅格；在 odom 坐标系（假 TF 里和 map 重合）
            half = int(3.0 / self.res)
            cx, cy = int((self.x - self.ox) / self.res), int((self.y - self.oy) / self.res)
            x0, y0 = max(0, cx - half), max(0, cy - half)
            x1, y1 = min(self.w, cx + half), min(self.h, cy + half)
            data = self.cost[y0:y1, x0:x1]
            m.header.frame_id = 'odom'
            m.info.origin.position.x, m.info.origin.position.y = self.ox + x0 * self.res, self.oy + y0 * self.res
        else:
            data = self.cost
            m.header.frame_id = 'map'
            m.info.origin.position.x, m.info.origin.position.y = self.ox, self.oy
        m.info.height, m.info.width = data.shape
        m.data = data.ravel().tolist()
        return m

    def deliver_map(self):
        from nav_msgs.msg import OccupancyGrid
        g = OccupancyGrid()
        g.info.width, g.info.height, g.info.resolution = self.w, self.h, self.res
        g.info.origin.position.x, g.info.origin.position.y = self.ox, self.oy
        g.data = self.grid.ravel().tolist()
        self.node.deliver('/map', g)

    def cloud_msg(self, ranges):
        """假 FAST-LIO 点云（body 坐标系）：墙上不同高度的点，外加离地很近的地面点（网页应该滤掉）。"""
        from sensor_msgs.msg import PointCloud2
        from tf2_ros import BODY_Z
        a = np.linspace(-math.pi, math.pi, len(ranges), endpoint=False)
        ok = np.isfinite(ranges) & (ranges < 12)
        r, a = ranges[ok], a[ok]
        k = 4
        rr = np.repeat(r, k) + np.random.normal(0, 0.02, len(r) * k)
        aa = np.repeat(a, k)
        z = np.random.uniform(0.05, 2.0, len(rr))
        wall = np.stack([rr * np.cos(aa), rr * np.sin(aa), z - BODY_Z], 1)
        g = np.random.uniform(0.5, 4.0, 600); ga = np.random.uniform(-math.pi, math.pi, 600)
        ground = np.stack([g * np.cos(ga), g * np.sin(ga), np.random.normal(0.0, 0.016, 600) - BODY_Z], 1)
        return PointCloud2(np.vstack([wall, ground]).astype(np.float32))

    def particles_msg(self):
        from nav2_msgs.msg import ParticleCloud
        s = 0.03 + 0.02 * abs(self.v)
        n = 500
        return ParticleCloud(zip(np.random.normal(self.x, s * 1.5, n), np.random.normal(self.y, s * 1.5, n),
                                 np.random.normal(self.th, math.radians(2), n), [1.0 / n] * n))

    def path_msg(self):
        from nav_msgs.msg import Path
        g = self.goal
        pts, out = [(self.x, self.y)] + list(g['path']), []
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):   # 和 Nav2 一样 5 cm 一个点
            n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.05))
            out += [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n)]
        return Path(out + [pts[-1]])

    def rosout(self, level, name, msg):
        from rcl_interfaces.msg import Log
        self.node.deliver('/rosout', Log(level, name, msg))


def run(node):
    """go2w_web 的 rclpy.spin(node)：按真实频率喂消息、处理指令和定时器。"""
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool

    os.makedirs(os.path.join(DATA, 'maps'), exist_ok=True)
    sim = Sim(node)
    node.fake_sim = sim
    sim.deliver_map()
    sim.publish_status()

    k, dt = 0, 0.05
    nxt = time.monotonic()
    map_sent = True
    while True:
        while True:
            try:
                sim.jobs.get_nowait()()
            except queue.Empty:
                break
        for cb in list(node._timers):
            cb()
        now = time.monotonic()
        sim.step(dt, now)
        sim.patrol_step(now)
        k += 1
        alive, nav = sim.any_ready(), sim.ready('navigation')
        # /map 是 latched：定位 / 建图起来时补发一次
        map_up = sim.loc_ready() or sim.ready('mapping')
        if map_up and not map_sent:
            sim.deliver_map()
        map_sent = map_up
        if alive and k % 2 == 0:  # /odom、/scan、点云、Nav2 反馈 10 Hz，和真机一致
            o = Odometry()
            o.header.stamp.sec, o.header.stamp.nanosec = sim.stamp()
            o.pose.pose.position.x, o.pose.pose.position.y = sim.x, sim.y
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(sim.th/2), math.cos(sim.th/2)
            o.twist.twist.linear.x = sim.v / sim.stamp_rate   # 和 planar_odom 一样按时间戳差分
            o.twist.twist.angular.z = sim.w_ / sim.stamp_rate
            node.deliver('/odom', o)
            s = LaserScan()
            s.header.frame_id = 'base_footprint'
            s.angle_min, s.angle_increment = -math.pi, 2*math.pi/360
            s.range_min, s.range_max = 0.1, 15.0
            ranges = sim.scan()
            s.ranges = ranges.tolist()
            node.deliver('/scan', s)
            node.deliver('/cloud_registered_body', sim.cloud_msg(ranges))
            sim.feedback(now)
            g, t = sim.goal, sim.teleop
            teleop_live = t and now - t[3] < 0.5
            if sim.bridge_up() and ((g and g['status'] == EXECUTING) or teleop_live):   # /cmd_vel：导航或遥控时一直在发
                tw = Twist(); tw.linear.x, tw.angular.z = (sim.v, sim.w_) if not teleop_live else (t[0], t[2])
                node.deliver('/cmd_vel', tw)
                if g and g['status'] == EXECUTING and k % 40 == 0:
                    sim.rosout(20, 'controller_server', 'Passing new path to controller.')
        if sim.bridge_up() and k % 2 == 1:  # cmd_bridge 的 ~/armed 和真机一样按自己的频率发
            b = Bool(); b.data = sim.armed; node.deliver('/go2w_cmd_bridge/armed', b)
        if alive and k % 10 == 0:  # 里程计健康 2 Hz
            b = Bool(); b.data = True; node.deliver('/odom_healthy', b)
        if nav and k % 10 == 5:   # 局部代价地图 2 Hz
            node.deliver('/local_costmap/costmap', sim.costmap_msg(local=True))
        if sim.loc_ready() and k % 10 == 5:   # 粒子 2 Hz
            node.deliver('/particle_cloud', sim.particles_msg())
        if nav and k % 20 == 0:   # 全局代价地图 1 Hz（内容不变，后端会去重）
            node.deliver('/global_costmap/costmap', sim.costmap_msg(local=False))
        if sim.loc_ready() and k % 20 == 0:  # /amcl_pose：假定位误差很小，走起来略大
            a = PoseWithCovarianceStamped()
            a.pose.pose.position.x, a.pose.pose.position.y = sim.x, sim.y
            sd = 0.03 + 0.02*abs(sim.v)
            a.pose.covariance[0] = a.pose.covariance[7] = sd*sd
            a.pose.covariance[35] = math.radians(1.5 + 2*abs(sim.w_))**2
            node.deliver('/amcl_pose', a)
        nxt += dt
        time.sleep(max(0.0, nxt - time.monotonic()))
