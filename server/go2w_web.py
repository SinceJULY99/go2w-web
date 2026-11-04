#!/usr/bin/env python3
"""Go2W 网页控制台后端。

一个进程里跑两件事：
  - rclpy 节点 go2w_web（域 43）：订阅地图 / scan / 里程计 / 健康状态 / 导航状态，查 TF，
    并代网页发初始位姿、导航目标、取消、解锁 / 上锁、调参
  - aiohttp：同一端口提供静态页面（web/static）和 WebSocket（/ws）

安全规则（P1）
  - 急停：取消所有导航目标 + 调 /go2w_cmd_bridge/arm(false)（cmd_bridge 立即发 StopMove 并上锁）。
    急停是锁存的：解除之前拒绝解锁和发目标；急停期间如果发现底盘被别处解锁，会再次上锁。
  - 心跳：网页每 0.5 s 发一次 ping。最近一个发过运动相关指令的网页是「控制方」；
    控制方心跳超过 --heartbeat 秒（默认 1.0）或连接断开，而底盘处于解锁或有导航目标时，
    取消导航并上锁。恢复后保持上锁，必须手动重新解锁。

WebSocket 协议
  浏览器 → 后端（JSON）：
    {"t": "ping", "ts"}
    {"t": "cmd", "id", "op", ...}，op = initial_pose | goal | cancel | arm | estop | release_estop | set_param
  后端 → 浏览器（JSON）：
    {"t": "pong"} {"t": "ack", "id", "ok", "msg"} {"t": "pose"} {"t": "state"} {"t": "task"} {"t": "log"}
  后端 → 浏览器（二进制）：u8 类型 + u32 元数据长度（小端）+ 元数据 JSON + 负载
    1 = 地图：负载是 zlib 压缩的 uint8 栅格（0..100 占用概率，255 未知），行 0 在 origin 一侧
    2 = scan：负载是 float32 [x, y, x, y, ...]，已经变换到 fixed 坐标系

运行（容器里）：python3 /go2w/web/server/go2w_web.py --port 8080
本地调试（Mac，假 ROS）：web/dev/run.sh
"""
import argparse
import asyncio
import json
import math
import os
import struct
import threading
import time
import zlib
from collections import deque

import numpy as np
import rclpy
import rclpy.time
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from aiohttp import WSMsgType, web
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog, ParticleCloud
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

import procs as P

# 二进制帧类型。代价地图 / 点云 / 粒子较重，只发给打开了对应图层的网页
T_MAP, T_SCAN, T_GCOST, T_LCOST, T_CLOUD, T_PART = 1, 2, 3, 4, 5, 6
HEAVY = {'gcost': T_GCOST, 'lcost': T_LCOST, 'cloud': T_CLOUD, 'particles': T_PART}
# 点云：离地高度范围（和 pcl2scan 一致）、最远距离、俯视体素、每帧最多点数、最高频率
CLOUD_Z, CLOUD_RANGE, CLOUD_VOXEL, CLOUD_MAX, CLOUD_DT = (0.30, 1.60), 12.0, 0.05, 4000, 0.2
PARTICLES_MAX, PARTICLES_DT = 300, 0.5
PLAN_MAX = 500
ROSOUT_PER_S = 30   # /rosout 转发限流（条 / 秒）
# 仓库的 data/（容器里是 /go2w/data）；本地调试时 run.sh 指向一份副本，免得写坏真实数据
DATA = os.environ.get('GO2W_DATA') or os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'data'))
# 档位进程起来后，看到这些节点才算「运行中」
MODE_NODES = {'mapping': ('slam_toolbox',), 'localization': ('amcl',), 'navigation': ('bt_navigator', 'go2w_cmd_bridge')}
START_TIMEOUT = 90.0
# 键盘遥控限速（网页按键给的速度再被这里夹一次，cmd_bridge 还会再限一次）
TELEOP_MAX = (0.5, 0.2, 1.0)
TELEOP_TIMEOUT = 0.3
INSPECT_DT = 0.5    # topic 查看器每条 topic 最多 2 Hz 刷新
ROS_LEVEL = {20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'ERROR'}
BASE_FRAME = 'base_footprint'
STALE = 3.0  # 秒；超过这么久没收到就当没有
BRIDGE = '/go2w_cmd_bridge'
# 网页上的滑条 → cmd_bridge 参数（都是 double）
PARAMS = {'vmax': 'max_vx', 'wmax': 'max_wz', 'tmo': 'cmd_timeout'}
# 和 RViz 2D Pose Estimate 相同的初始协方差
INIT_COV_XY, INIT_COV_YAW = 0.25, 0.06853891945200942
# 行为树里表示「在做恢复」的节点
RECOVERY_NODES = ('Spin', 'BackUp', 'Wait', 'DriveOnHeading')
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
RESULT = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled', GoalStatus.STATUS_ABORTED: 'aborted'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pack(kind, meta, payload=b''):
    j = json.dumps(meta, separators=(',', ':')).encode()
    return struct.pack('<BI', kind, len(j)) + j + payload


def secs(d):
    return d.sec + d.nanosec * 1e-9


def rot_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]], dtype=np.float32)


def grid_blob(kind, m, place):
    """OccupancyGrid → 二进制帧。place = (x, y, yaw)：栅格原点在 fixed 坐标系下的位置和朝向。"""
    info = m.info
    grid = np.asarray(m.data, dtype=np.int8).astype(np.int16)
    grid[grid < 0] = 255
    meta = {'w': info.width, 'h': info.height, 'res': info.resolution,
            'ox': place[0], 'oy': place[1], 'oyaw': place[2], 'frame': m.header.frame_id}
    return pack(kind, meta, zlib.compress(grid.astype(np.uint8).tobytes(), 6)), grid


def roll_pitch(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


MATCH_TOL = 0.10  # m：scan 点离地图障碍物这么近就算「对上了」


class Rate:
    """最近 2 秒的消息频率。"""

    def __init__(self):
        self.stamps = []

    def tick(self):
        now = time.monotonic()
        self.stamps.append(now)
        while self.stamps and now - self.stamps[0] > 2.0:
            self.stamps.pop(0)

    def hz(self):
        now = time.monotonic()
        s = [t for t in self.stamps if now - t <= 2.0]
        return round(len(s) / 2.0, 1)


class SpeedCheck:
    """诊断用：同一段 /odom 位移，分别除以「消息时间戳差」和「本机收到消息的时间差」。

    planar_odom 的 twist 用的是时间戳差；如果雷达时间戳走得比真实时间快，位置是对的，
    速度却会整体偏小。rate = 时间戳走过的时间 / 真实走过的时间，正常应接近 1。
    """

    WINDOW = 0.5  # 秒（真实时间）

    def __init__(self):
        self.buf = []  # (wall, stamp, x, y, yaw)

    def add(self, wall, stamp, x, y, yaw):
        if self.buf and stamp <= self.buf[-1][1]:
            return
        self.buf.append((wall, stamp, x, y, yaw))
        while self.buf and wall - self.buf[0][0] > 2.0:
            self.buf.pop(0)

    def result(self):
        if len(self.buf) < 2 or time.monotonic() - self.buf[-1][0] > 0.5:
            return None
        last = self.buf[-1]
        first = next((s for s in self.buf if last[0] - s[0] <= self.WINDOW), self.buf[0])
        dw, ds = last[0] - first[0], last[1] - first[1]
        if dw < 0.2 or ds <= 0:
            return None
        c, s = math.cos(last[4]), math.sin(last[4])
        dx, dy = last[2] - first[2], last[3] - first[3]
        fwd, side = c * dx + s * dy, -s * dx + c * dy
        dyaw = math.atan2(math.sin(last[4] - first[4]), math.cos(last[4] - first[4]))
        return {'wall': [round(fwd / dw, 3), round(side / dw, 3), round(dyaw / dw, 3)],
                'stamp': [round(fwd / ds, 3), round(side / ds, 3), round(dyaw / ds, 3)],
                'rate': round(ds / dw, 3)}


class SysStats:
    """宿主机 CPU / 内存（容器用 host 网络，/proc/stat 和 /proc/meminfo 反映的是整机）。"""

    def __init__(self):
        self.prev = None

    def read(self):
        try:
            return self._read()
        except OSError:  # 读不到就显示 --，不影响其他状态
            return {'cpu': None, 'mem': None, 'memTotal': None}

    def _read(self):
        with open('/proc/stat') as f:
            v = [int(x) for x in f.readline().split()[1:]]
        idle, total = v[3] + v[4], sum(v)
        cpu = None
        if self.prev:
            dt, di = total - self.prev[1], idle - self.prev[0]
            cpu = round(100.0 * (1 - di / dt), 1) if dt > 0 else None
        self.prev = (idle, total)
        mem = {}
        with open('/proc/meminfo') as f:
            for line in f:
                k, val = line.split(':')
                mem[k] = int(val.split()[0])
        total_g = mem['MemTotal'] / 1048576
        used_g = (mem['MemTotal'] - mem.get('MemAvailable', mem['MemFree'])) / 1048576
        return {'cpu': cpu, 'mem': round(used_g, 2), 'memTotal': round(total_g, 1)}


class Bridge(Node):
    # 节点名 → 界面上的名字；按 launch 文件里的 name= 对应
    NODES = [
        ('livox_lidar_publisher', '雷达驱动'),
        ('fast_lio', 'FAST-LIO'),
        ('planar_odom', 'planar_odom'),
        ('pcl2scan_slam', 'pcl2scan'),
        ('amcl', 'AMCL'),
        ('slam_toolbox', 'slam_toolbox'),
        ('bt_navigator', 'Nav2'),
        ('go2w_cmd_bridge', 'cmd_bridge'),
    ]

    def __init__(self):
        super().__init__('go2w_web')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.rates = {k: Rate() for k in ('/scan', '/odom', '/odom_healthy', '/go2w_cmd_bridge/armed')}
        self.lock = threading.Lock()   # 保护 goal / statuses / bt_running（rclpy 线程写，asyncio 线程读）
        self.events = deque(maxlen=300)  # (级别, 节点, 消息) → 推给网页日志（本节点事件 + /rosout）
        self.rosout_budget, self.rosout_t, self.rosout_dropped = ROSOUT_PER_S, time.monotonic(), 0
        self.wants = lambda layer: False  # 由 Server 替换：有没有网页打开了这个图层（重的数据没人看就不算）

        # P2 图层：最新的二进制帧和版本号，广播循环发现版本变了才发
        self.blobs = {k: None for k in HEAVY.values()}
        self.vers = {k: 0 for k in HEAVY.values()}
        self.grid_raw = {}       # 代价地图原始字节，内容没变就不重发
        self.gcost = None        # 全局代价地图 (栅格, res, ox, oy, w, h)，给网页判断目标能不能去
        self.cloud_t = self.part_t = 0.0
        self.plan, self.plan_ver = [], 0
        self.cmd = (0.0, 0.0, 0.0, 0.0)  # /cmd_vel (vx, vy, wz, 收到时间)

        # P3
        self.fb_any = {}         # 所有目标的反馈（含 RViz、patrol 发的）：uuid → (剩余距离, 用时, 恢复次数)
        self.patrol = {'A': None, 'B': None, 'loops': 0, 'target': None, 'legs': 0, 'state': None}
        self.map_current = None  # map_server 正在用的地图名（读它的 yaml_filename 参数）
        self.inspect = {}        # topic → {'sub', 'text', 't', 'ver', 'rate'}；只在有网页查看时订阅
        self.ros_ops = deque()   # 要在 rclpy 线程里做的事（建 / 删订阅），由定时器执行
        self.create_timer(0.1, self.run_ros_ops)

        self.map_blob, self.map_ver = None, 0
        self.near_occ = None     # 地图障碍物膨胀 MATCH_TOL 后的布尔栅格，算匹配率用
        self.map_info = None
        self.amcl = None         # (位置 σ m, 航向 σ °, 收到时间)
        self.match = None        # scan / map 匹配率 %，平滑过
        self.scan_blob, self.scan_ver = None, 0
        self.odom = None
        self.speed = SpeedCheck()
        self.rate_warned = 0.0
        self.healthy, self.healthy_t = None, 0.0
        self.armed, self.armed_t = None, 0.0
        self.fixed = 'map'

        self.goal = None        # 网页发出的当前目标（见 track_goal）
        self.statuses = {}      # 所有 navigate_to_pose 目标的状态：uuid → status（含 RViz、patrol 发的）
        self.bt_running = set()
        self.params = {}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Bool, '/odom_healthy', self.on_healthy, 10)
        self.create_subscription(Bool, f'{BRIDGE}/armed', self.on_armed, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_status, latched)
        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self.on_bt, 10)
        # AMCL 的 /amcl_pose 是 transient_local，只在机器人移动后更新；用 latched 订阅才能一连上就拿到最近一次
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched)
        # P2：代价地图（costmap_2d 用 transient_local 发布）、路径、速度指令、粒子、点云、所有节点的日志
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: self.on_costmap(m, T_GCOST), latched)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: self.on_costmap(m, T_LCOST), latched)
        self.create_subscription(Path, '/plan', self.on_plan, 1)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(ParticleCloud, '/particle_cloud', self.on_particles, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self.on_rosout, 100)
        # 所有 navigate_to_pose 目标的反馈（RViz、patrol 发的也有），任务卡片显示外部目标的进度用
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback', self.on_any_feedback, 10)

        self.patrol_pub = self.create_publisher(PoseStamped, '/patrol_pose', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.loadmap_cli = self.create_client(LoadMap, '/map_server/load_map')
        self.mapparam_cli = self.create_client(GetParameters, '/map_server/get_parameters')

        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cancel_cli = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.arm_cli = self.create_client(SetBool, f'{BRIDGE}/arm')
        self.pget_cli = self.create_client(GetParameters, f'{BRIDGE}/get_parameters')
        self.pset_cli = self.create_client(SetParameters, f'{BRIDGE}/set_parameters')

    def event(self, level, msg):
        """记日志并推给网页。不会抛异常：停车等安全动作不能因为打日志失败而中断。

        rclpy 的同一处调用（同一行代码）不能先后用不同级别打日志，否则抛
        ValueError('Logger severity cannot be changed between calls.')，所以三个级别各占一行。
        """
        self.events.append((level, 'go2w_web', msg))
        try:
            if level == 'ERROR':
                self.get_logger().error(msg)
            elif level == 'WARN':
                self.get_logger().warn(msg)
            else:
                self.get_logger().info(msg)
        except Exception:
            pass

    # ---------- 订阅回调（rclpy 线程） ----------
    def on_map(self, m):
        info = m.info
        grid = np.asarray(m.data, dtype=np.int8).astype(np.int16)
        grid[grid < 0] = 255
        meta = {'w': info.width, 'h': info.height, 'res': info.resolution,
                'ox': info.origin.position.x, 'oy': info.origin.position.y,
                'oyaw': yaw_of(info.origin.orientation), 'frame': m.header.frame_id}
        self.map_blob = pack(T_MAP, meta, zlib.compress(grid.astype(np.uint8).tobytes(), 6))
        occ = (grid >= 65) & (grid != 255)
        occ = occ.reshape(info.height, info.width)
        near = occ.copy()
        for _ in range(max(1, round(MATCH_TOL / info.resolution))):   # 4 邻域膨胀
            n2 = near.copy()
            n2[1:, :] |= near[:-1, :]; n2[:-1, :] |= near[1:, :]; n2[:, 1:] |= near[:, :-1]; n2[:, :-1] |= near[:, 1:]
            near = n2
        self.near_occ, self.map_info = near, (info.resolution, info.origin.position.x, info.origin.position.y, info.width, info.height)
        self.map_ver += 1
        self.get_logger().info(f'map {info.width}x{info.height} @ {info.resolution:.3f} m')

    def on_scan(self, m):
        self.rates['/scan'].tick()
        tfm = self.lookup(m.header.frame_id)
        if tfm is None:
            return
        tx, ty, th = tfm
        r = np.asarray(m.ranges, dtype=np.float32)
        a = m.angle_min + np.arange(r.size, dtype=np.float32) * m.angle_increment
        ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
        r, a = r[ok], a[ok] + th
        xy = np.empty((r.size, 2), dtype=np.float32)
        xy[:, 0] = tx + r * np.cos(a)
        xy[:, 1] = ty + r * np.sin(a)
        self.scan_blob = pack(T_SCAN, {'n': int(r.size), 'frame': self.fixed}, xy.tobytes())
        self.scan_ver += 1
        # 匹配率：只在 map 坐标系下有意义（退回 odom 时地图和 scan 本来就不对齐）
        if self.fixed == 'map' and self.near_occ is not None and r.size >= 20:
            res, ox, oy, w, h = self.map_info
            gx = np.floor((xy[:, 0] - ox) / res).astype(int)
            gy = np.floor((xy[:, 1] - oy) / res).astype(int)
            inside = (gx >= 0) & (gy >= 0) & (gx < w) & (gy < h)
            if inside.sum() >= 20:
                pct = 100.0 * self.near_occ[gy[inside], gx[inside]].mean()
                self.match = pct if self.match is None else 0.8 * self.match + 0.2 * pct

    def on_amcl(self, m):
        c = m.pose.covariance
        self.amcl = (math.sqrt(max(0.0, (c[0] + c[7]) / 2)), math.degrees(math.sqrt(max(0.0, c[35]))), time.monotonic())

    # ---------- P2 图层 ----------
    def place(self, m):
        """栅格原点在 fixed 坐标系下的 (x, y, yaw)。局部代价地图在 odom 下，要经 TF 转到 map。"""
        o = m.info.origin
        f = (0.0, 0.0, 0.0) if m.header.frame_id in ('', self.fixed) else self.lookup(m.header.frame_id)
        if f is None:
            return None
        tx, ty, th = f
        c, s = math.cos(th), math.sin(th)
        return (tx + c*o.position.x - s*o.position.y, ty + s*o.position.x + c*o.position.y, th + yaw_of(o.orientation))

    def on_costmap(self, m, kind):
        raw = np.asarray(m.data, dtype=np.int8).tobytes()
        # 全局代价地图大多数时候不变（1 Hz 发布），内容相同就不重发；局部的随机器人移动，每次都发
        if kind == T_GCOST and self.grid_raw.get(kind) == raw:
            return
        place = self.place(m)
        if place is None:
            return
        self.blobs[kind], _ = grid_blob(kind, m, place)
        self.grid_raw[kind] = raw
        self.vers[kind] += 1

    def on_plan(self, m):
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        frame = m.header.frame_id or 'map'
        if pts and frame != self.fixed:
            f = self.lookup(frame)
            if f is None:
                return
            c, s = math.cos(f[2]), math.sin(f[2])
            pts = [(f[0] + c*x - s*y, f[1] + s*x + c*y) for x, y in pts]
        step = max(1, math.ceil(len(pts) / PLAN_MAX))
        keep = pts[::step] + ([pts[-1]] if pts and (len(pts) - 1) % step else [])
        self.plan = [[round(x, 3), round(y, 3)] for x, y in keep]
        self.plan_ver += 1

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z, time.monotonic())

    def on_particles(self, m):
        now = time.monotonic()
        if not self.wants('particles') or now - self.part_t < PARTICLES_DT or self.fixed != 'map':
            return
        self.part_t = now
        ps = m.particles
        step = max(1, math.ceil(len(ps) / PARTICLES_MAX))
        arr = np.array([(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation)) for p in ps[::step]],
                       dtype=np.float32).reshape(-1, 3)
        self.blobs[T_PART] = pack(T_PART, {'n': len(arr), 'total': len(ps)}, arr.tobytes())
        self.vers[T_PART] += 1

    def on_cloud(self, m):
        """FAST-LIO 点云 → 机器人坐标系按离地高度过滤 → 5 cm 俯视体素（每格留最高点）→ fixed 坐标系 xy + 高度档。"""
        now = time.monotonic()
        if not self.wants('cloud') or now - self.cloud_t < CLOUD_DT:
            return
        self.cloud_t = now
        try:
            t = self.tf.lookup_transform(BASE_FRAME, m.header.frame_id, rclpy.time.Time()).transform
        except Exception:
            return
        pose = self.lookup(BASE_FRAME)
        off = {f.name: f.offset for f in m.fields}
        if pose is None or not all(k in off for k in 'xyz'):
            return
        dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                       'offsets': [off['x'], off['y'], off['z']], 'itemsize': m.point_step})
        a = np.frombuffer(bytes(m.data), dtype=dt, count=m.width * m.height)
        P = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float32)
        P = P[np.isfinite(P).all(1)]
        tr = t.translation
        P = P @ rot_matrix(t.rotation).T + np.array([tr.x, tr.y, tr.z], dtype=np.float32)
        z = P[:, 2]
        P = P[(z >= CLOUD_Z[0]) & (z <= CLOUD_Z[1]) & (P[:, 0]**2 + P[:, 1]**2 <= CLOUD_RANGE**2)]
        if len(P):
            P = P[np.argsort(-P[:, 2])]
            k = np.floor(P[:, :2] / CLOUD_VOXEL).astype(np.int64)
            _, idx = np.unique(k[:, 0] * 1000003 + k[:, 1], return_index=True)
            P = P[idx]
            if len(P) > CLOUD_MAX:
                P = P[np.random.choice(len(P), CLOUD_MAX, replace=False)]
        x0, y0, th = pose
        c, s = math.cos(th), math.sin(th)
        xy = np.empty((len(P), 2), dtype=np.float32)
        xy[:, 0] = x0 + c*P[:, 0] - s*P[:, 1]
        xy[:, 1] = y0 + s*P[:, 0] + c*P[:, 1]
        band = np.clip((P[:, 2] - CLOUD_Z[0]) / (CLOUD_Z[1] - CLOUD_Z[0]) * 6, 0, 5).astype(np.uint8)
        self.blobs[T_CLOUD] = pack(T_CLOUD, {'n': len(xy), 'z': CLOUD_Z}, xy.tobytes() + band.tobytes())
        self.vers[T_CLOUD] += 1

    def on_rosout(self, m):
        level = m.level[0] if isinstance(m.level, (bytes, bytearray)) else m.level   # Log.level 是 byte 类型
        lv = ROS_LEVEL.get(level)
        if lv is None or m.name == 'go2w_web':   # DEBUG 不转；本节点的事件已经单独推送
            return
        if m.name == 'go2w_patrol':
            self.track_patrol(lv, m.msg)
        now = time.monotonic()
        if now - self.rosout_t >= 1.0:
            if self.rosout_dropped:
                self.events.append(('WARN', 'go2w_web', f'/rosout 太多，过去 1 s 少转发了 {self.rosout_dropped} 条'))
            self.rosout_budget, self.rosout_t, self.rosout_dropped = ROSOUT_PER_S, now, 0
        if self.rosout_budget <= 0:
            self.rosout_dropped += 1
            return
        self.rosout_budget -= 1
        self.events.append((lv, m.name, m.msg))

    # ---------- P3 ----------
    def track_patrol(self, lv, text):
        """patrol.py 没有状态 topic，按它的日志跟踪进度（文案见 scripts/patrol.py）。"""
        p = self.patrol
        if text.startswith('→ 去 '):
            p['target'], p['state'] = text[4], 'running'
        elif text.startswith('到达 ') and '第 ' in text:
            try:
                p['legs'] = int(text.split('第 ')[1].split(' ')[0])
            except (IndexError, ValueError):
                pass
        elif text.startswith('巡逻完成'):
            p['state'] = 'done'
        elif text.startswith('开始巡逻'):
            p['state'], p['legs'] = 'running', 0
        elif lv == 'ERROR':
            p['state'] = 'error'

    def on_any_feedback(self, msg):
        fb = msg.feedback
        self.fb_any[bytes(msg.goal_id.uuid)] = (fb.distance_remaining, secs(fb.navigation_time), fb.number_of_recoveries,
                                                secs(fb.estimated_time_remaining), time.monotonic())

    def run_ros_ops(self):
        while self.ros_ops:
            self.ros_ops.popleft()()

    def inspect_on(self, topic):
        """开始查看某个 topic：按它的类型动态订阅（在 rclpy 线程里建，避免和执行器抢）。"""
        types = dict(self.get_topic_names_and_types())
        if topic not in types or not types[topic]:
            return False
        cls = get_message(types[topic][0])
        rec = {'sub': None, 'text': None, 't': 0.0, 'ver': 0, 'rate': Rate(), 'type': types[topic][0]}
        self.inspect[topic] = rec

        def create():
            if self.inspect.get(topic) is rec:
                rec['sub'] = self.create_subscription(cls, topic, lambda m: self.on_inspect(rec, m), qos_profile_sensor_data)
        self.ros_ops.append(create)
        return True

    def inspect_off(self, topic):
        rec = self.inspect.pop(topic, None)
        if rec:
            self.ros_ops.append(lambda: rec['sub'] is not None and self.destroy_subscription(rec['sub']))

    def on_inspect(self, rec, m):
        rec['rate'].tick()
        now = time.monotonic()
        if now - rec['t'] < INSPECT_DT:
            return
        rec['t'] = now
        try:
            text = message_to_yaml(m, truncate_length=12)
        except Exception as e:
            text = f'（这条消息转不成文字：{e!r}）'
        rec['text'] = text if len(text) < 8000 else text[:8000] + '\n…（太长，已截断）'
        rec['ver'] += 1

    def publish_patrol_point(self, x, y, th):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        self.patrol_pub.publish(m)

    def publish_cmd(self, v, vy, w):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(v), float(vy), float(w)
        self.cmd_pub.publish(t)

    def load_map(self, name):
        req = LoadMap.Request()
        req.map_url = f'/go2w/data/maps/{name}.yaml'
        return self.loadmap_cli.call_async(req)

    def fetch_map_current(self):
        if not self.mapparam_cli.service_is_ready():
            self.map_current = None
            return
        req = GetParameters.Request()
        req.names = ['yaml_filename']

        def done(fut):
            try:
                v = fut.result().values[0].string_value
                self.map_current = os.path.basename(v)[:-5] if v.endswith('.yaml') else (v or None)
            except Exception:
                pass
        self.mapparam_cli.call_async(req).add_done_callback(done)

    def on_odom(self, m):
        self.rates['/odom'].tick()
        t = m.twist.twist
        self.odom = (t.linear.x, t.linear.y, t.angular.z)
        p, st = m.pose.pose, m.header.stamp
        self.speed.add(time.monotonic(), st.sec + st.nanosec * 1e-9, p.position.x, p.position.y, yaw_of(p.orientation))

    def on_healthy(self, m):
        self.rates['/odom_healthy'].tick()
        self.healthy, self.healthy_t = m.data, time.monotonic()

    def on_armed(self, m):
        self.rates['/go2w_cmd_bridge/armed'].tick()
        self.armed, self.armed_t = m.data, time.monotonic()

    def on_status(self, m):
        with self.lock:
            self.statuses = {bytes(s.goal_info.goal_id.uuid): s.status for s in m.status_list}

    def on_bt(self, m):
        with self.lock:
            for e in m.event_log:
                if e.current_status == 'RUNNING':
                    self.bt_running.add(e.node_name)
                else:
                    self.bt_running.discard(e.node_name)

    def on_feedback(self, msg):
        fb = msg.feedback
        with self.lock:
            g = self.goal
            if g is None or bytes(msg.goal_id.uuid) != g['id']:
                return
            g['remaining'] = fb.distance_remaining
            g['total'] = max(g['total'] or 0.0, fb.distance_remaining)
            g['nav_time'] = secs(fb.navigation_time)
            g['eta'] = secs(fb.estimated_time_remaining)
            g['recoveries'] = fb.number_of_recoveries

    def on_result(self, gid, fut):
        try:
            status = fut.result().status
        except Exception as e:  # 结果拿不到也要结束这个任务
            status, err = GoalStatus.STATUS_ABORTED, e
        else:
            err = None
        res = RESULT.get(status, 'aborted')
        with self.lock:
            g = self.goal
            if g is None or g['id'] != gid:
                return
            g['status'], g['end'] = res, time.monotonic()
            if res == 'succeeded':
                g['remaining'] = 0.0
        p = g['pose']
        if res == 'succeeded':
            self.event('INFO', f"到达目标 ({p[0]:.2f}, {p[1]:.2f})，用时 {g['nav_time'] or 0:.1f} s")
        elif res == 'canceled':
            self.event('INFO', '导航目标已取消')
        else:
            self.event('ERROR', f'导航失败（Nav2 放弃了这个目标）{": " + repr(err) if err else ""}')

    def on_params(self, fut):
        try:
            vals = fut.result().values
        except Exception:
            return
        for key, v in zip(PARAMS, vals):
            if v.type == ParameterType.PARAMETER_DOUBLE:
                self.params[key] = v.double_value
            elif v.type == ParameterType.PARAMETER_INTEGER:
                self.params[key] = float(v.integer_value)

    # ---------- 指令（asyncio 线程调用；返回 rclpy future） ----------
    def publish_initial_pose(self, x, y, th):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = INIT_COV_XY
        cov[35] = INIT_COV_YAW
        m.pose.covariance = cov
        self.initpose_pub.publish(m)

    def send_goal(self, x, y, th):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        return self.nav.send_goal_async(g, feedback_callback=self.on_feedback)

    def track_goal(self, handle, pose):
        gid = bytes(handle.goal_id.uuid)
        with self.lock:
            self.goal = {'id': gid, 'pose': pose, 't0': time.monotonic(), 'status': 'active', 'end': None,
                         'remaining': None, 'total': None, 'nav_time': None, 'eta': None, 'recoveries': 0}
        handle.get_result_async().add_done_callback(lambda f: self.on_result(gid, f))

    def cancel_all(self):
        """取消所有 navigate_to_pose 目标（零 uuid + 零时间戳 = 全部），包括 RViz、patrol 发的。"""
        return self.cancel_cli.call_async(CancelGoal.Request())

    def call_arm(self, on):
        req = SetBool.Request()
        req.data = bool(on)
        return self.arm_cli.call_async(req)

    def fetch_params(self):
        if not self.pget_cli.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = list(PARAMS.values())
        self.pget_cli.call_async(req).add_done_callback(self.on_params)

    def set_param(self, key, value):
        p = Parameter()
        p.name = PARAMS[key]
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        req = SetParameters.Request()
        req.parameters = [p]
        return self.pset_cli.call_async(req)

    # ---------- 查询（asyncio 线程调用，只读） ----------
    def lookup(self, frame):
        """frame 在 fixed 坐标系下的 (x, y, yaw)。没有 map 时退回 odom（例如只跑了里程计）。"""
        for fixed in ('map', 'odom'):
            try:
                t = self.tf.lookup_transform(fixed, frame, rclpy.time.Time())
            except Exception:
                continue
            self.fixed = fixed
            tr = t.transform
            return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)
        return None

    def pose(self):
        p = self.lookup(BASE_FRAME)
        v = self.odom or (0.0, 0.0, 0.0)
        if p is None:
            return {'t': 'pose', 'valid': False}
        sc = self.speed.result()
        # 走起来（> 0.1 m/s）且时间戳走速偏离真实时间 5% 以上时，最多每 10 s 报一次
        if sc and abs(sc['wall'][0]) > 0.1 and abs(sc['rate'] - 1) > 0.05 and time.monotonic() - self.rate_warned > 10:
            self.rate_warned = time.monotonic()
            self.get_logger().warn(f"/odom 时间戳走速是真实时间的 {sc['rate']:.2f} 倍："
                                   f"twist 速度 {v[0]:.2f} m/s，按真实时间算 {sc['wall'][0]:.2f} m/s")
        # 机身姿态：planar_odom 把机器人 IMU 的俯仰 / 横滚发成 base_footprint → base_link
        tilt = None
        try:
            q = self.tf.lookup_transform(BASE_FRAME, 'base_link', rclpy.time.Time()).transform.rotation
            rp = roll_pitch(q)
            tilt = [round(math.degrees(rp[0]), 2), round(math.degrees(rp[1]), 2)]
        except Exception:
            pass
        return {'t': 'pose', 'valid': True, 'frame': self.fixed,
                'x': round(p[0], 4), 'y': round(p[1], 4), 'th': round(p[2], 4),
                'v': round(v[0], 3), 'vy': round(v[1], 3), 'w': round(v[2], 3), 'speed': sc, 'tilt': tilt,
                # 实际下发的速度指令（velocity_smoother 之后）；0.5 s 没有新指令按零算
                'cmd': [round(c, 3) for c in self.cmd[:3]] if time.monotonic() - self.cmd[3] < 0.5 else [0.0, 0.0, 0.0]}

    def loc_quality(self):
        # 机器人不动时 AMCL 不更新 /amcl_pose，协方差也不会变，所以一直用最近一次的值
        a = self.amcl
        return {'std': round(a[0], 4) if a else None, 'yaw': round(a[1], 2) if a else None,
                'match': round(self.match, 1) if self.match is not None and self.fixed == 'map' else None}

    def is_armed(self):
        return bool(self.armed) if time.monotonic() - self.armed_t < STALE else None

    def nav_active(self):
        with self.lock:
            return any(s in ACTIVE for s in self.statuses.values()) or bool(self.goal and self.goal['status'] == 'active')

    def task(self):
        """任务卡片：网页发的目标显示详细进度；别处（RViz、patrol）发的只显示「外部目标」。"""
        with self.lock:
            g = dict(self.goal) if self.goal else None
            running = set(self.bt_running)
            foreign = [k for k, s in self.statuses.items() if s in ACTIVE and (not g or k != g['id'])]
        recovering = any(n in RECOVERY_NODES or n.startswith('Clear') for n in running)
        phase = 'recover' if recovering else 'follow' if 'FollowPath' in running else 'plan'
        if g:
            if g['status'] != 'active' and time.monotonic() - g['end'] > 5.0:
                with self.lock:
                    if self.goal and self.goal['id'] == g['id']:
                        self.goal = None
                g = None
        if g:
            return {'type': 'goal', 'status': g['status'],
                    'phase': phase if g['status'] == 'active' else 'arrived' if g['status'] == 'succeeded' else g['status'],
                    'goal': {'x': g['pose'][0], 'y': g['pose'][1], 'th': g['pose'][2]},
                    'remaining': g['remaining'], 'total': g['total'], 'eta': g['eta'],
                    'elapsed': g['nav_time'] if g['nav_time'] is not None else time.monotonic() - g['t0'],
                    'recoveries': g['recoveries']}
        p = self.patrol
        patrolling = p['state'] == 'running'
        if foreign or patrolling:
            # 别处发的目标没有我们自己的 action 句柄，进度来自 /navigate_to_pose/_action/feedback
            fb = self.fb_any.get(foreign[0]) if foreign else None
            fresh = fb and time.monotonic() - fb[4] < 3.0
            t = {'status': 'active', 'phase': phase if foreign else 'arrived',
                 'remaining': fb[0] if fresh else None, 'elapsed': fb[1] if fresh else None,
                 'recoveries': fb[2] if fresh else 0, 'eta': fb[3] if fresh else None, 'total': None}
            if patrolling:
                tgt = p[p['target']] if p['target'] in ('A', 'B') else None
                t.update(type='patrol', leg=p['target'], legs=p['legs'], loops=p['loops'],
                         goal={'x': tgt[0], 'y': tgt[1], 'th': tgt[2]} if tgt else None)
            else:
                t['type'] = 'external'
            return t
        return None

    def state(self, sys_stats, procs_status=None, transitions=None):
        now = time.monotonic()
        names = set(self.get_node_names())
        if 'bt_navigator' not in names:
            self.forget_nav()
        nodes = [{'id': n, 'name': label, 'up': n in names} for n, label in self.NODES]
        profiles = self.profiles(names, procs_status, transitions)
        topics = []
        for name, types in sorted(self.get_topic_names_and_types()):
            if name.startswith('/parameter_events'):
                continue
            r = self.rates.get(name)
            topics.append({'name': name, 'type': types[0] if types else '', 'pubs': self.count_publishers(name),
                           'hz': r.hz() if r else None})
        return {
            't': 'state',
            'fixed': self.fixed,
            'odomOk': bool(self.healthy) and now - self.healthy_t < STALE,
            'armed': self.is_armed() if 'go2w_cmd_bridge' in names else None,
            'navReady': self.nav.server_is_ready(),
            'nodes': nodes, 'profiles': profiles, 'topics': topics,
            'hasMap': self.map_blob is not None,
            'params': dict(self.params),
            'loc': self.loc_quality(),
            'patrol': {k: self.patrol[k] for k in ('A', 'B', 'loops', 'state')},
            'mapCurrent': self.map_current,
            'sys': sys_stats,
        }

    def forget_nav(self):
        """Nav2 不在了（停止或崩溃）：不会再有结果回来，清掉残留的任务状态，否则任务卡片和心跳判断会一直以为在导航。"""
        with self.lock:
            self.statuses, self.bt_running = {}, set()
            self.grid_raw = {}   # 重启后代价地图内容可能完全一样，也要重新发给网页
            g = self.goal
            if g and g['status'] == 'active':
                g['status'], g['end'] = 'aborted', time.monotonic()
            else:
                g = None
        if g:
            self.event('ERROR', 'Nav2 已经不在运行，当前导航目标作废')

    def profiles(self, names, procs_status=None, transitions=None):
        """档位状态：进程在 + 关键节点在 = 运行中；进程在、节点还没起来 = 启动中；
        拿不到进程状态时（本地没有 docker.sock）只看节点。"""
        profiles = {}
        for m in P.MODES:
            nodes_up = all(n in names for n in MODE_NODES[m])
            if procs_status is None:
                st = nodes_up and not (m == 'localization' and 'bt_navigator' in names)
            else:
                st = procs_status.get(m, False)
            profiles[m] = ('run' if nodes_up or procs_status is None else 'starting') if st else 'stop'
        profiles['patrol'] = 'run' if 'go2w_patrol' in names or (procs_status or {}).get('patrol') else 'stop'
        profiles['bag'] = 'run' if any(n.startswith('rosbag2_recorder') for n in names) or (procs_status or {}).get('bag') else 'stop'
        for k, v in (transitions or {}).items():   # 网页刚发起的启停，进程状态还没跟上
            profiles[k] = v
        if profiles['patrol'] == 'stop' and self.patrol['state'] == 'running':
            self.patrol['state'] = None             # patrol.py 被停掉时不会打「巡逻完成」
        return profiles


class Server:
    def __init__(self, node, static_dir, heartbeat):
        self.node, self.static_dir, self.heartbeat = node, static_dir, heartbeat
        self.clients = {}       # ws → {'remote', 'last_ping'}
        self.owner = None       # 控制方：最近一个发过运动相关指令的网页
        self.estop, self.estop_t = False, 0.0
        self.pending = set()
        node.wants = self.wants   # 点云、粒子没人看就不算，省扩展坞的 CPU
        self.sys = SysStats()
        # P3：进程管理。本地调试（GO2W_FAKE_PROCS=1）换成假模拟器里的进程
        if os.environ.get('GO2W_FAKE_PROCS'):
            from go2w_fake_procs import FakeProcs
            self.procs = FakeProcs(node)
        else:
            self.procs = P.DockerProcs()
        self.procs_status = None      # 最近一次进程检查结果；None = 查不到（没有 docker.sock）
        self.transitions = {}         # 网页发起、还没完成的启停：档位 → 'starting' / 'stopping'
        self.starting = {}            # 档位 → 开始启动的时间（启动超时 / 失败检测）
        self.busy = False             # 同一时间只做一个启停操作
        self.map_choice = None        # 没有定位 / 导航在跑时，网页选好的「下次启动用的地图」
        self.teleop = None            # (vx, vy, wz, 收到时间)；None = 没有在遥控
        self.maps = P.list_maps(os.path.join(DATA, 'maps'))
        self.bags = P.list_bags(os.path.join(DATA, 'bags'))

    # ---------- 工具 ----------
    def aw(self, fut, timeout=3.0):
        """把 rclpy future 变成可 await 的 asyncio future（回调在 rclpy 线程里触发）。"""
        loop = asyncio.get_running_loop()
        af = loop.create_future()

        def done(f):
            def settle():
                if af.done():
                    return
                try:
                    af.set_result(f.result())
                except Exception as e:
                    af.set_exception(e)
            loop.call_soon_threadsafe(settle)
        fut.add_done_callback(done)
        return asyncio.wait_for(af, timeout)

    async def disarm_and_cancel(self, reason):
        """停车：取消所有导航 + 上锁。服务不可用时照样尝试另一半，并如实报告。"""
        n, errors = self.node, []
        if self.teleop is not None:   # 遥控也立即停
            self.teleop = None
            n.publish_cmd(0.0, 0.0, 0.0)
        if n.cancel_cli.service_is_ready():
            try:
                await self.aw(n.cancel_all(), 2.0)
            except Exception as e:
                errors.append(f'取消导航失败：{e!r}')
        if n.arm_cli.service_is_ready():
            try:
                r = await self.aw(n.call_arm(False), 2.0)
                if r.success:
                    n.armed, n.armed_t = False, time.monotonic()   # 不等下一条 ~/armed，cmd_bridge 可能马上就被停掉
                else:
                    errors.append(f'上锁失败：{r.message}')
            except Exception as e:
                errors.append(f'上锁失败：{e!r}')
        else:
            errors.append('cmd_bridge 不在线，无法上锁（底盘没有在执行导航指令时不影响）')
        if errors:
            n.event('ERROR', f'{reason}：' + '；'.join(errors))
        else:
            n.event('WARN', f'{reason}：已取消导航并上锁')
        return errors

    # ---------- 指令 ----------
    def remote(self, ws):
        return self.clients.get(ws, {}).get('remote', '?')

    async def run_cmd(self, ws, d):
        try:
            ok, text = await self.handle_cmd(ws, d)
        except Exception as e:
            self.node.get_logger().error(f'cmd {d.get("op")}: {e!r}')
            ok, text = False, f'后端出错：{e!r}'
        if not ws.closed:
            await ws.send_str(json.dumps({'t': 'ack', 'id': d.get('id'), 'ok': ok, 'msg': text}))

    async def handle_cmd(self, ws, d):
        n, op = self.node, d.get('op')
        if op in ('goal', 'arm', 'cancel', 'estop'):
            self.owner = ws
        try:
            if op == 'estop':
                # 先停车，再记日志
                self.estop, self.estop_t = True, time.monotonic()
                errors = await self.disarm_and_cancel(f'网页急停（{self.remote(ws)}）')
                return (False, '急停已锁存，但：' + '；'.join(errors)) if errors else (True, '已急停：导航已取消，底盘已上锁')
            if op == 'release_estop':
                self.estop = False
                n.event('INFO', '急停已解除（底盘仍上锁）')
                return True, '急停已解除，底盘仍处于上锁'
            if op == 'initial_pose':
                if 'amcl' not in n.get_node_names():
                    return False, '定位（AMCL）没有运行，无法设置初始位姿'
                n.publish_initial_pose(d['x'], d['y'], d['th'])
                n.event('INFO', f"初始位姿 ({d['x']:.2f}, {d['y']:.2f}, {math.degrees(d['th']):.0f}°)")
                return True, '初始位姿已发布'
            if op == 'goal':
                if self.estop:
                    return False, '急停中：先解除急停'
                if self.prof('patrol') != 'stop':
                    return False, '巡逻进行中：先停止巡逻再发目标'
                if not n.nav.server_is_ready():
                    return False, 'Nav2 没有运行（navigate_to_pose 不可用）'
                h = await self.aw(n.send_goal(d['x'], d['y'], d['th']), 3.0)
                if not h.accepted:
                    return False, 'Nav2 拒绝了这个目标'
                n.track_goal(h, (d['x'], d['y'], d['th']))
                n.event('INFO', f"导航目标 ({d['x']:.2f}, {d['y']:.2f}, {math.degrees(d['th']):.0f}°)")
                return True, '目标已下发' if n.is_armed() else '目标已下发，底盘上锁中，解锁后才会运动'
            if op == 'cancel':
                # 巡逻中只取消当前目标的话，patrol.py 会重试，机器人又走起来：要连巡逻一起停
                if self.prof('patrol') != 'stop':
                    await self.procs.stop_patrol()
                    await asyncio.sleep(0.5)
                    await self.refresh_procs()
                    n.event('INFO', '已停止巡逻')
                if not n.cancel_cli.service_is_ready():
                    return False, 'Nav2 没有运行'
                await self.aw(n.cancel_all(), 2.0)
                return True, '已取消导航'
            if op == 'arm':
                on = bool(d.get('on'))
                if on and self.estop:
                    return False, '急停中：先解除急停'
                if not n.arm_cli.service_is_ready():
                    return False, 'cmd_bridge 没有运行（导航没启动？）'
                r = await self.aw(n.call_arm(on), 2.0)
                if r.success:
                    n.armed, n.armed_t = on, time.monotonic()
                n.event('WARN' if on else 'INFO', f"{'解锁' if on else '上锁'}：{r.message}（{self.remote(ws)}）")
                return r.success, r.message  # 以 cmd_bridge 的回复为准
            if op == 'set_param':
                key = d.get('key')
                if key not in PARAMS:
                    return False, f'不支持的参数：{key}'
                if not n.pset_cli.service_is_ready():
                    return False, 'cmd_bridge 没有运行'
                r = await self.aw(n.set_param(key, d['value']), 2.0)
                res = r.results[0]
                if res.successful:
                    n.params[key] = float(d['value'])
                    n.event('INFO', f"{BRIDGE} {PARAMS[key]} = {float(d['value']):.2f}")
                    return True, '参数已更新'
                return False, f'参数没有生效：{res.reason}'
            if op == 'set_mode':
                return self.set_mode(d.get('mode'), d.get('map'))
            if op == 'patrol_point':
                k = d.get('k')
                if k not in ('A', 'B'):
                    return False, '巡逻点只能是 A 或 B'
                if self.prof('patrol') != 'stop':
                    return False, '巡逻进行中：patrol.py 不接受新点，先停止巡逻再改'
                n.patrol[k] = (float(d['x']), float(d['y']), float(d['th']))
                return True, f'巡逻点 {k} 已设置'
            if op == 'start_patrol':
                self.owner = ws
                return await self.start_patrol(int(d.get('loops', 0)))
            if op == 'stop_patrol':
                self.transitions['patrol'] = 'stopping'
                try:
                    await self.procs.stop_patrol()
                    await asyncio.sleep(0.5)      # patrol.py 要先取消目标再退出
                    await self.refresh_procs()
                finally:
                    self.transitions.pop('patrol', None)
                n.event('INFO', '已停止巡逻')
                return True, '已停止巡逻（patrol.py 会取消当前导航目标）'
            if op == 'bag_start':
                return await self.bag_start(d.get('name'), d.get('topics') or [])
            if op == 'bag_stop':
                self.transitions['bag'] = 'stopping'
                try:
                    await self.procs.stop_bag()
                    await self.refresh_procs()
                    self.bags = P.list_bags(os.path.join(DATA, 'bags'))
                finally:
                    self.transitions.pop('bag', None)
                n.event('INFO', '录包已停止')
                return True, '录包已停止'
            if op == 'save_map':
                return await self.save_map(d.get('name', ''))
            if op == 'load_map':
                return await self.load_map(d.get('name', ''))
            return False, f'不支持的操作：{op}'
        except asyncio.TimeoutError:
            return False, '对方没有响应（超时）'
        except (KeyError, TypeError, ValueError) as e:
            return False, f'指令格式不对：{e!r}'

    # ---------- P3：运行档位、巡逻、录包、地图 ----------
    MODE_NAME = {'mapping': '建图', 'localization': '定位', 'navigation': '导航'}

    def prof(self, key):
        """当前档位状态（和推给网页的算法一致）：stop / starting / run / stopping。"""
        n = self.node
        return n.profiles(set(n.get_node_names()), self.procs_status, self.transitions).get(key, 'stop')

    async def refresh_procs(self):
        """启停之后马上重查进程，不等 2 s 一次的轮询：否则状态滞后，按钮会显示成相反的操作。"""
        try:
            self.procs_status = await self.procs.status()
        except Exception:
            self.procs_status = None

    def background(self, coro):
        t = asyncio.create_task(coro)
        self.pending.add(t)
        t.add_done_callback(self.pending.discard)

    def set_mode(self, mode, map_name=None):
        n = self.node
        if self.busy:
            return False, '上一个启停还没完成，稍等几秒'
        if mode is not None and mode not in P.MODES:
            return False, f'没有这个档位：{mode}'
        cur = [m for m in P.MODES if self.prof(m) != 'stop']
        if mode is not None and cur == [mode]:
            return True, f'{self.MODE_NAME[mode]}已经在运行'
        if mode in ('localization', 'navigation'):
            names = [m['name'] for m in self.maps]
            map_name = map_name or self.map_choice or n.map_current or ('lab_20261021' if 'lab_20261021' in names else (names[0] if names else None))
            if map_name not in names:
                return False, 'data/maps 里没有可用的地图'
        if not cur and mode is None:
            return True, '没有在运行的档位'
        self.busy = True
        self.background(self.do_mode(mode, cur, map_name))
        if mode is None:
            return True, '正在停止……'
        return True, f"正在启动{self.MODE_NAME[mode]}{'（地图 ' + map_name + '）' if map_name else ''}：FAST-LIO 初始化时机器人要保持静止"

    async def do_mode(self, mode, cur, map_name):
        n = self.node
        try:
            if cur or self.prof('patrol') != 'stop':
                for m in cur:
                    self.transitions[m] = 'stopping'
                if n.is_armed() or n.nav_active():
                    await self.disarm_and_cancel('切换运行档位')
                ok = await self.procs.stop_modes()
                await self.refresh_procs()
                for m in cur:
                    self.transitions.pop(m, None)
                    self.starting.pop(m, None)
                if ok:
                    n.event('INFO', f"已停止{'、'.join(self.MODE_NAME[m] for m in cur) or '巡逻'}")
                else:
                    n.event('ERROR', '还有导航进程没退出，没有继续启动新档位。可以再点一次，或在扩展坞上执行 nav.sh stop')
                    return
            if mode:
                self.transitions[mode] = 'starting'
                self.starting[mode] = time.monotonic()
                await self.procs.start_mode(mode, map_name)
                await asyncio.sleep(0.3)
                await self.refresh_procs()
                n.event('INFO', f"启动{self.MODE_NAME[mode]}{'（地图 ' + map_name + '）' if map_name else ''}，"
                                f'日志在 data/logs/{mode}.log')
                if mode != 'mapping':
                    self.map_choice = None
        except Exception as e:
            self.transitions.pop(mode, None)
            self.starting.pop(mode, None)
            n.event('ERROR', f'启停失败：{e!r}')
        finally:
            self.busy = False

    async def start_patrol(self, loops):
        n, p = self.node, self.node.patrol
        if self.estop:
            return False, '急停中：先解除急停'
        if self.prof('navigation') != 'run':
            return False, '导航没有运行：先启动「导航」'
        if not (p['A'] and p['B']):
            return False, '先在地图上设置 A、B 两个巡逻点'
        if self.prof('patrol') != 'stop':
            return False, '巡逻已经在运行'
        if not 0 <= loops <= 1000:
            return False, '往返次数要在 0~1000 之间（0 = 一直跑）'
        self.transitions['patrol'] = 'starting'
        p.update(loops=loops, legs=0, target=None, state='starting')
        await self.procs.start_patrol(loops)
        await asyncio.sleep(0.3)
        await self.refresh_procs()
        self.background(self.feed_patrol())
        armed = n.is_armed()
        return True, '巡逻已启动' + ('' if armed else '：底盘锁定中，patrol.py 会等到解锁后才出发')

    async def feed_patrol(self):
        """等 patrol.py 订阅上 /patrol_pose，再按 A、B 的顺序发点（它只接受前两个点）。"""
        n, p = self.node, self.node.patrol
        try:
            t0 = time.monotonic()
            while n.patrol_pub.get_subscription_count() < 1:
                if time.monotonic() - t0 > 20:
                    n.event('ERROR', 'patrol.py 20 s 内没有起来，看 data/logs/patrol.log：' + P.tail(os.path.join(DATA, 'logs', 'patrol.log'), 6))
                    p['state'] = None
                    return
                await asyncio.sleep(0.2)
            await asyncio.sleep(0.5)
            for k in ('A', 'B'):
                n.publish_patrol_point(*p[k])
                await asyncio.sleep(0.3)
            n.event('INFO', f"巡逻点已发给 patrol.py：{'一直往返' if p['loops'] == 0 else '往返 %d 次' % p['loops']}")
        finally:
            self.transitions.pop('patrol', None)

    async def bag_start(self, name, topics):
        name = name or time.strftime('web_%m%d_%H%M%S')
        if not P.SAFE_NAME.match(name):
            return False, '包名只能用字母、数字、下划线和减号'
        bad = [t for t in topics if not P.SAFE_TOPIC.match(t)]
        if not topics or bad:
            return False, f"topic 写法不对：{' '.join(bad) or '（空）'}"
        if self.prof('bag') != 'stop':
            return False, '已经在录包了'
        if any(b['name'] == name for b in self.bags):
            return False, f'data/bags 里已经有 {name}，换个名字'
        self.transitions['bag'] = 'starting'
        try:
            await self.procs.start_bag(name, topics)
            await asyncio.sleep(0.3)
            await self.refresh_procs()
        finally:
            self.transitions.pop('bag', None)
        self.node.event('INFO', f"开始录包 {name}：{' '.join(topics)}")
        return True, f'开始录包：data/bags/{name}'

    async def save_map(self, name):
        if self.prof('mapping') != 'run':
            return False, '保存地图需要「建图」正在运行'
        if not P.SAFE_NAME.match(name):
            return False, '地图名只能用字母、数字、下划线和减号'
        if any(m['name'] == name for m in self.maps):
            return False, f'已经有同名地图 {name}，换个名字（不会覆盖）'
        self.node.event('INFO', f'保存地图 {name}……')
        if await self.procs.save_map(name):
            self.maps = P.list_maps(os.path.join(DATA, 'maps'))
            self.node.event('INFO', f'地图已保存：data/maps/{name}')
            return True, f'地图已保存：{name}'
        return False, '保存失败：' + (P.tail(os.path.join(DATA, 'logs', 'save_map.log'), 6) or '看 data/logs/save_map.log')

    async def load_map(self, name):
        n = self.node
        if name not in [m['name'] for m in self.maps]:
            return False, f'data/maps 里没有 {name}'
        if not n.loadmap_cli.service_is_ready():
            self.map_choice = name
            return True, f'下次启动定位 / 导航时用 {name}'
        if n.nav_active():
            await self.aw(n.cancel_all(), 2.0)
        r = await self.aw(n.load_map(name), 10.0)
        if r.result != 0:
            return False, f'map_server 加载失败（错误码 {r.result}）'
        n.map_current = name
        n.event('WARN', f'已切换到地图 {name}：请重新设置初始位姿')
        return True, f'已切换到 {name}，请重新设置初始位姿'

    async def procs_poll(self):
        """每 2 s 查一次进程；判断启动完成 / 失败 / 超时；每 6 s 刷新地图、录包列表和当前地图。"""
        n, k = self.node, 0
        while True:
            k += 1
            try:
                self.procs_status = await self.procs.status()
            except Exception as e:
                if self.procs_status is not None or k == 1:
                    n.get_logger().warn(f'查不到进程状态（{e!r}），运行档位只按节点判断')
                self.procs_status = None
            names = set(n.get_node_names())
            for m, t0 in list(self.starting.items()):
                age = time.monotonic() - t0
                if all(x in names for x in MODE_NODES[m]):
                    self.starting.pop(m); self.transitions.pop(m, None)
                    n.event('INFO', f'{self.MODE_NAME[m]}已启动')
                elif self.procs_status is not None and not self.procs_status.get(m) and age > 4:
                    self.starting.pop(m); self.transitions.pop(m, None)
                    n.event('ERROR', f'{self.MODE_NAME[m]}启动失败，进程退出了。data/logs/{m}.log 最后几行：\n'
                            + P.tail(os.path.join(DATA, 'logs', f'{m}.log'), 8))
                elif age > START_TIMEOUT:
                    self.starting.pop(m); self.transitions.pop(m, None)
                    n.event('ERROR', f'{self.MODE_NAME[m]} {START_TIMEOUT:.0f} s 内没有起来，看 data/logs/{m}.log')
            if k % 3 == 1:
                self.maps = P.list_maps(os.path.join(DATA, 'maps'))
                self.bags = P.list_bags(os.path.join(DATA, 'bags'))
                n.fetch_map_current()
            await asyncio.sleep(2.0)

    # ---------- 键盘遥控 ----------
    def on_teleop(self, ws, d):
        """网页按住按键时每 0.1 s 发一次；后端 0.3 s 收不到就发零速并停止发布。"""
        n, now = self.node, time.monotonic()
        refuse = '急停中' if self.estop else '底盘锁定中，先解锁' if not n.is_armed() else None
        if refuse:
            if now - self.clients[ws].get('teleop_warned', 0) > 2:
                self.clients[ws]['teleop_warned'] = now
                self.background(ws.send_str(json.dumps({'t': 'toast', 'level': 'err', 'msg': f'遥控没有生效：{refuse}'})))
            return
        self.owner = ws
        if self.teleop is None and n.nav_active():
            self.background(self.aw(n.cancel_all(), 2.0))   # 遥控和导航不能同时发 /cmd_vel
            n.event('WARN', '开始键盘遥控：已取消导航目标')
        c = lambda v, m: max(-m, min(m, float(v)))
        self.teleop = (c(d.get('v', 0), TELEOP_MAX[0]), c(d.get('vy', 0), TELEOP_MAX[1]), c(d.get('w', 0), TELEOP_MAX[2]), now)

    def teleop_tick(self):
        if self.teleop is None:
            return
        if time.monotonic() - self.teleop[3] < TELEOP_TIMEOUT and not self.estop:
            self.node.publish_cmd(*self.teleop[:3])
        else:
            self.node.publish_cmd(0.0, 0.0, 0.0)   # 松手 / 断线：零速一次，然后不再发布，把 /cmd_vel 还给 Nav2
            self.teleop = None

    # ---------- topic 查看 ----------
    def set_inspect(self, ws, topic, on):
        watching = self.clients[ws].setdefault('inspect', set())
        if on and topic not in watching:
            if topic not in self.node.inspect and not self.node.inspect_on(topic):
                self.background(ws.send_str(json.dumps({'t': 'msg', 'topic': topic, 'text': None, 'err': '这个 topic 现在没有发布者'})))
                return
            watching.add(topic)
            self.node.inspect[topic]['sent'] = -1
        elif not on and topic in watching:
            watching.discard(topic)
            if not any(topic in c.get('inspect', ()) for c in self.clients.values()):
                self.node.inspect_off(topic)

    # ---------- HTTP / WebSocket ----------
    async def index(self, _req):
        return web.FileResponse(os.path.join(self.static_dir, 'index.html'))

    async def ws(self, req):
        ws = web.WebSocketResponse(max_msg_size=1 << 20)
        await ws.prepare(req)
        self.clients[ws] = {'remote': req.remote, 'last_ping': time.monotonic(), 'layers': set()}
        self.node.get_logger().info(f'client connected: {req.remote} ({len(self.clients)})')
        try:
            if self.node.map_blob:
                await ws.send_bytes(self.node.map_blob)
            if self.node.plan:
                await ws.send_str(json.dumps({'t': 'plan', 'pts': self.node.plan}))
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    d = json.loads(msg.data)
                except ValueError:
                    continue
                if d.get('t') == 'ping':
                    self.clients[ws]['last_ping'] = time.monotonic()
                    await ws.send_str(json.dumps({'t': 'pong', 'ts': d.get('ts')}))
                elif d.get('t') == 'sub':
                    # 网页打开 / 关闭了哪些重图层；新打开的立刻补发最近一帧（全局代价地图可能很久才变一次）
                    new = {x for x in d.get('layers', []) if x in HEAVY}
                    added = new - self.clients[ws]['layers']
                    self.clients[ws]['layers'] = new
                    for layer in added:
                        blob = self.node.blobs[HEAVY[layer]]
                        if blob:
                            await ws.send_bytes(blob)
                elif d.get('t') == 'teleop':
                    self.on_teleop(ws, d)
                elif d.get('t') == 'inspect' and isinstance(d.get('topic'), str):
                    self.set_inspect(ws, d['topic'], bool(d.get('on')))
                elif d.get('t') == 'cmd':
                    # 指令可能要等几秒（Nav2 接受目标、服务回复），单独跑，不耽误这条连接上的心跳
                    t = asyncio.create_task(self.run_cmd(ws, d))
                    self.pending.add(t)                    # asyncio 只弱引用任务，要自己持有
                    t.add_done_callback(self.pending.discard)
        finally:
            for topic in list(self.clients.get(ws, {}).get('inspect', ())):
                self.set_inspect(ws, topic, False)
            self.clients.pop(ws, None)
            self.node.get_logger().info(f'client gone: {req.remote} ({len(self.clients)})')
            if ws is self.owner:
                self.owner = None
                await self.lose_owner(f'控制方网页断开（{req.remote}）')
        return ws

    async def lose_owner(self, reason):
        n = self.node
        if n.is_armed() or n.nav_active():
            await self.disarm_and_cancel(reason)

    def wants(self, layer):
        return any(layer in c['layers'] for c in list(self.clients.values()))

    async def send_all(self, data, layer=None):
        """发给所有网页；layer 不为空时只发给打开了这个图层的网页。"""
        async def one(ws):
            try:
                if isinstance(data, bytes):
                    await asyncio.wait_for(ws.send_bytes(data), 0.5)
                else:
                    await asyncio.wait_for(ws.send_str(data), 0.5)
            except Exception:
                self.clients.pop(ws, None)
                await ws.close()
        targets = [ws for ws, c in list(self.clients.items()) if layer is None or layer in c['layers']]
        if targets:
            await asyncio.gather(*(one(ws) for ws in targets))

    async def safety(self):
        """心跳和急停的周期检查，和推送分开，不受网络慢的客户端影响。"""
        n, k = self.node, 0
        while True:
            await asyncio.sleep(0.1)
            k += 1
            try:
                self.teleop_tick()
                o = self.owner
                if o is not None and o in self.clients:
                    gap = time.monotonic() - self.clients[o]['last_ping']
                    if gap > self.heartbeat:
                        self.owner = None
                        await self.lose_owner(f"控制方网页心跳丢失 {gap:.1f} s（{self.clients[o]['remote']}）")
                # 急停期间底盘被别处解锁（nav.sh arm 等）→ 再次上锁。
                # 只认急停 1 s 之后收到的「已解锁」，避免把上锁之前的旧状态当成又被解锁
                if self.estop and n.is_armed() and n.armed_t > self.estop_t + 1.0 and k % 5 == 0:
                    await self.disarm_and_cancel('急停期间底盘被解锁')
                if k % 50 == 0:
                    n.fetch_params()
            except Exception as e:  # 安全检查不能因为一次异常停掉
                n.get_logger().error(f'safety: {e!r}')

    async def broadcast(self):
        n = self.node
        map_ver = scan_ver = plan_ver = 0
        vers = {k: 0 for k in HEAVY.values()}
        k = 0
        while True:
            await asyncio.sleep(0.05)  # 20 Hz 节拍
            k += 1
            # 没有客户端时也记下地图版本：新客户端连上时 ws() 已经单独发过当前地图
            new_map = n.map_ver != map_ver
            map_ver = n.map_ver
            if not self.clients:
                n.events.clear()
                continue
            try:
                await self.send_all(json.dumps(n.pose()))
                if k % 2 == 0 and n.scan_ver != scan_ver:  # scan 最多 10 Hz
                    scan_ver = n.scan_ver
                    await self.send_all(n.scan_blob)
                if new_map:
                    await self.send_all(n.map_blob)
                if k % 4 == 0:  # 任务 5 Hz
                    await self.send_all(json.dumps({'t': 'task', 'task': n.task()}))
                if n.plan_ver != plan_ver:
                    plan_ver = n.plan_ver
                    await self.send_all(json.dumps({'t': 'plan', 'pts': n.plan}))
                for layer, kind in HEAVY.items():
                    if n.vers[kind] != vers[kind]:
                        vers[kind] = n.vers[kind]
                        await self.send_all(n.blobs[kind], layer)
                if n.events:  # 日志攒一批一起发（/rosout 可能一秒几十条）
                    items = []
                    while n.events:
                        items.append(n.events.popleft())
                    await self.send_all(json.dumps({'t': 'logs', 'items': items}))
                if k % 10 == 0:  # 状态 2 Hz
                    st = n.state(self.sys.read(), self.procs_status, self.transitions)
                    st.update(estop=self.estop, busy=self.busy, maps=self.maps, bags=self.bags, mapChoice=self.map_choice,
                              procsKnown=self.procs_status is not None, teleop=self.teleop is not None)
                    await self.send_all(json.dumps(st))
                # topic 查看器：只发给正在看这个 topic 的网页
                for topic, rec in list(n.inspect.items()):
                    if rec['text'] is not None and rec['ver'] != rec.get('sent'):
                        rec['sent'] = rec['ver']
                        msg = json.dumps({'t': 'msg', 'topic': topic, 'type': rec['type'], 'hz': rec['rate'].hz(), 'text': rec['text']})
                        for ws, c in list(self.clients.items()):
                            if topic in c.get('inspect', ()):
                                try:
                                    await asyncio.wait_for(ws.send_str(msg), 0.5)
                                except Exception:
                                    pass
            except Exception as e:  # 别让一次异常停掉推送
                n.get_logger().warn(f'broadcast: {e!r}')

    def app(self):
        @web.middleware
        async def no_cache(req, handler):  # 开发期改完前端直接刷新就生效
            resp = await handler(req)
            resp.headers['Cache-Control'] = 'no-store'
            return resp

        app = web.Application(middlewares=[no_cache])
        app.router.add_get('/', self.index)
        app.router.add_get('/ws', self.ws)
        for sub in ('css', 'js', 'img'):   # img/：机器人照片（可选，见 static/img/README.md）
            if os.path.isdir(os.path.join(self.static_dir, sub)):
                app.router.add_static(f'/{sub}/', os.path.join(self.static_dir, sub))

        async def start(a):
            a['tasks'] = [asyncio.create_task(self.broadcast()), asyncio.create_task(self.safety()),
                          asyncio.create_task(self.procs_poll())]

        async def stop(a):
            for t in a['tasks']:
                t.cancel()

        app.on_startup.append(start)
        app.on_cleanup.append(stop)
        return app


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--static', default=os.path.join(here, '..', 'static'))
    ap.add_argument('--heartbeat', type=float, default=1.0, help='控制方网页心跳超时（秒）')
    args = ap.parse_args()

    rclpy.init()
    node = Bridge()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    node.get_logger().info(f'go2w_web on http://0.0.0.0:{args.port}，心跳超时 {args.heartbeat:.1f} s')
    try:
        web.run_app(Server(node, os.path.abspath(args.static), args.heartbeat).app(), host='0.0.0.0', port=args.port,
                    print=None, handle_signals=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
