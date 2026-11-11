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
import shutil
import struct
import tempfile
import threading
import time
import uuid
import zlib
from collections import deque

import numpy as np
import rclpy
import rclpy.time
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from aiohttp import WSMsgType, web
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog, ParticleCloud
from nav2_msgs.srv import ClearCostmapExceptRegion, LoadMap
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

import camera as C
import mapfile
import procs as P
import robot_ctrl as R

# 二进制帧类型。代价地图 / 点云 / 粒子较重，只发给打开了对应图层的网页
T_MAP, T_SCAN, T_GCOST, T_LCOST, T_CLOUD, T_PART = 1, 2, 3, 4, 5, 6
T_CAM = 7   # 摄像头一帧 H.264（Annex-B），只发给正在看的那一个网页
HEAVY = {'gcost': T_GCOST, 'lcost': T_LCOST, 'cloud': T_CLOUD, 'particles': T_PART}
# 点云：离地高度范围（和 pcl2scan 一致）、最远距离、俯视体素、每帧最多点数、最高频率
CLOUD_Z, CLOUD_RANGE, CLOUD_VOXEL, CLOUD_MAX, CLOUD_DT = (0.30, 1.60), 12.0, 0.05, 4000, 0.2
PARTICLES_MAX, PARTICLES_DT = 300, 0.5
PLAN_MAX = 500
ROSOUT_PER_S = 30   # /rosout 转发限流（条 / 秒）
# 仓库的 data/（容器里是 /go2w/data）；本地调试时 run.sh 指向一份副本，免得写坏真实数据
DATA = os.environ.get('GO2W_DATA') or os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'data'))
# 录包上传到常驻服务器 166:~/go2w_bags。密钥在 data/ssh（不随 sync.sh 同步）；166 上这把钥匙被 rrsync 限制成只能往 go2w_bags 写
UPLOAD_DEST = os.environ.get('GO2W_UPLOAD_DEST', 'evoengine@192.168.1.166:')
UPLOAD_LABEL = os.environ.get('GO2W_UPLOAD_LABEL', '166:~/go2w_bags')
# 录包容量上限：data/bags 总量超过 BAG_LIMIT_GB，或磁盘剩余不到 BAG_MIN_FREE_GB，就从最旧的开始删「已上传到 166」的包；
# 没上传的不自动删，只报警并拒绝开始新的录包；正在录的包自己就超过上限（或磁盘快满）时自动停止录包
BAG_LIMIT_GB = float(os.environ.get('GO2W_BAG_LIMIT_GB', '100'))
BAG_MIN_FREE_GB = float(os.environ.get('GO2W_BAG_MIN_FREE_GB', '10'))
ROLLBACK_SECONDS = 20         # 回溯录制的长度
ROLLBACK_RETRY = 30.0         # 回溯进程退出后，隔多久再拉起来
ROLLBACK_SAVE_TIMEOUT = 20.0  # 等存包完成的最长时间
# 卡住自动保存：导航目标没结束、底盘已解锁、没急停，odom 位置在 STUCK_MOVE 内、朝向在 STUCK_TURN 内
# 持续 STUCK_SECONDS 就算卡住；再等 STUCK_DELAY 存回溯包（stuck_ 开头），两次自动保存至少隔 STUCK_COOLDOWN
STUCK_SECONDS = float(os.environ.get('GO2W_STUCK_SECONDS', '8'))
STUCK_MOVE, STUCK_TURN = 0.08, 0.15
STUCK_DELAY, STUCK_COOLDOWN = 2.0, 60.0
# 档位进程起来后，看到这些节点才算「运行中」
MODE_NODES = {'mapping': ('slam_toolbox',), 'localization': ('amcl',), 'navigation': ('bt_navigator', 'go2w_cmd_bridge')}
START_TIMEOUT = 90.0
BRIDGE_RETRY = 10.0   # 遥控速度桥退出后，隔这么久再重启
# 键盘遥控限速（网页按键给的速度再被这里夹一次，cmd_bridge 还会再限一次）
TELEOP_MAX = (1.0, 0.2, 1.2)
TELEOP_TIMEOUT = 0.3
INSPECT_DT = 0.5    # topic 查看器每条 topic 最多 2 Hz 刷新
ROS_LEVEL = {20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'ERROR'}
BASE_FRAME = 'base_footprint'
STALE = 3.0  # 秒；超过这么久没收到就当没有
# 路线启动时离第 1 站不到这么远，就当已在第 1 站、直接去第 2 站：这么短的路径 RotationShim 不会先转正
# （forward_sampling_distance 0.5），MPPI 只给出很小的速度，Go2W 不执行，会一直卡住
ROUTE_NEAR_START = 0.5
# 多点任务：Nav2 报未到达（差几厘米到 goal checker 0.15 m 判不过、收尾蹭不进去）时，
# 机器人离该站不超过这么远就当作到了，继续下一站；更远才终止任务
ROUTE_ARRIVE_TOL = 0.4
# 地图显示方向（只转网页画面，不改地图坐标）：按地图名存在 data/map_views.json，大家看到的方向一样。
# 建图时还没有地图名，用 MAPPING_VIEW 暂存，存图时带给新地图
MAPPING_VIEW = '_mapping'
BRIDGE = '/go2w_cmd_bridge'
# 网页上的滑条 → cmd_bridge 参数（都是 double）
PARAMS = {'vmax': 'max_vx', 'wmax': 'max_wz', 'tmo': 'cmd_timeout'}
# 和 RViz 2D Pose Estimate 相同的初始协方差
COSTMAP_REFILL = 1.5   # s：清完代价地图后等障碍层按当前点云重新标记（全局 1 Hz 更新）
INIT_COV_XY, INIT_COV_YAW = 0.25, 0.06853891945200942
# 重定位微调（网页「应用到 AMCL」）：用户已经对准了，σ 取 5 cm / 2°，粒子不撒开
FINE_COV_XY, FINE_COV_YAW = 0.05 ** 2, math.radians(2) ** 2
# 自动定位（globloc.py，子进程跑）：用最近 AUTOLOC_SCANS 帧 scan（机器人静止时叠在一起）在整张地图上搜。
# 定位 / 导航档位启动完成后自动跑一次（GO2W_AUTO_LOCALIZE=0 关掉）；网页「自动定位」按钮随时可以再跑
AUTOLOC_SCANS = 5
AUTOLOC_STILL = (0.03, 0.05)   # 静止判定：|v| < 0.03 m/s 且 |wz| < 0.05 rad/s
AUTOLOC_TIMEOUT = 60.0
AUTOLOC_ON_START = os.environ.get('GO2W_AUTO_LOCALIZE', '1') != '0'
# 刚站起来（策略刚接管）机身还在晃，雷达高度也和趴着时不一样：策略 ready 后再稳这么久，只用这之后的 scan
AUTOLOC_SETTLE = 2.0
AUTOLOC_SETTLE_WAIT = 15.0   # s：最多等这么久机器人站稳、静止，等不到就报失败
# 运控（go2w_ctrl）站起 / 趴下，见 robot_ctrl.py。GO2W_CTRL=0 关掉（不查站没站起来，也不自动趴下）
CTRL_ON = os.environ.get('GO2W_CTRL', '1') != '0'
CTRL_STALE = 2.0       # s：状态 topic 10 Hz，超过这么久没收到就当不知道
CTRL_WAIT = 3.0        # s：每一步切换（插值走完之后）最多等这么久，和同事的 sport_cli up 一致
CTRL_INTERP = 2.0      # s：FixStand 从趴姿插值到站姿的时间（config.yaml 的 ts 末项）
CTRL_RESEND = 0.5      # s：请求还没被接受就重发（运控 0.2 s 后作废没取走的请求，DDS 也可能丢）
# 网页站起来（或跑过任务）的机器狗，空闲这么久就原地趴下；0 = 不自动趴下
IDLE_LIE = float(os.environ.get('GO2W_IDLE_LIE', '30'))
PREP_HOLD_TTL = 5.0    # s：网页自检进行中每 2 s 发一次 prep_hold，期间不算空闲
STILL_V, STILL_W = 0.05, 0.1   # 趴下前要求里程计速度低于这些（m/s、rad/s）
# 行为树里表示「在做恢复」的节点
RECOVERY_NODES = ('Spin', 'BackUp', 'Wait', 'DriveOnHeading')
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
RESULT = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled', GoalStatus.STATUS_ABORTED: 'aborted'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class StuckWatch:
    """导航中机器人一直停在原地（位置、朝向都基本没变）就报一次；动起来或任务结束后重新计时。"""
    def __init__(self, seconds=STUCK_SECONDS, move=STUCK_MOVE, turn=STUCK_TURN):
        self.seconds, self.move, self.turn = seconds, move, turn
        self.anchor, self.fired = None, False

    def update(self, now, pose, active):
        """pose = odom 下的 (x, y, yaw) 或 None。刚判定卡住时返回已经停了多少秒，其余时候返回 None。"""
        if not active or pose is None:
            self.anchor, self.fired = None, False
            return None
        a = self.anchor
        if (a is None or math.hypot(pose[0] - a[0], pose[1] - a[1]) > self.move
                or abs(math.atan2(math.sin(pose[2] - a[2]), math.cos(pose[2] - a[2]))) > self.turn):
            self.anchor, self.fired = (pose[0], pose[1], pose[2], now), False
            return None
        if not self.fired and now - a[3] >= self.seconds:
            self.fired = True
            return now - a[3]
        return None


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
        self.map_current = None  # map_server 正在用的地图名（读它的 yaml_filename 参数）
        self.inspect = {}        # topic → {'sub', 'text', 't', 'ver', 'rate'}；只在有网页查看时订阅
        self.ros_ops = deque()   # 要在 rclpy 线程里做的事（建 / 删订阅），由定时器执行
        self.create_timer(0.1, self.run_ros_ops)

        self.map_blob, self.map_ver = None, 0
        self.map_file = None     # 网页上显示的是 data/maps 里直接读的哪张图（导航没开时）；None = 来自 /map
        self.near_occ = None    # 地图障碍物膨胀 MATCH_TOL 后的布尔栅格，算匹配率用
        self.map_info = None
        self.map_loaded = None   # 网页「切换地图」最近加载的地图名（map_server 重启后作废）
        self.amcl = None         # (位置 σ m, 航向 σ °, 收到时间)
        self.match = None        # scan / map 匹配率 %，平滑过
        self.scan_blob, self.scan_ver = None, 0
        self.map_raw = None      # 自动定位用：(OccupancyGrid 原始 int8 栅格 h×w, res, ox, oy)
        self.scans_base = deque(maxlen=AUTOLOC_SCANS)   # 自动定位用：(收到时间, base_footprint 下的 N×2 点, 收到时是否静止)
        self.odom = None
        self.odom_pose = None    # (x, y, yaw, 收到时间)，odom 坐标系；卡住检测用
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

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.loadmap_cli = self.create_client(LoadMap, '/map_server/load_map')
        self.mapparam_cli = self.create_client(GetParameters, '/map_server/get_parameters')

        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.cancel_cli = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.arm_cli = self.create_client(SetBool, f'{BRIDGE}/arm')
        self.pget_cli = self.create_client(GetParameters, f'{BRIDGE}/get_parameters')
        self.pset_cli = self.create_client(SetParameters, f'{BRIDGE}/set_parameters')
        # 出发前清代价地图：MID360 往下只看到约 7°，机器人身边贴地的一圈是盲区，人站过留下的障碍标记清不掉，
        # 起点落在障碍里 Nav2 就报「Either of the start or goal pose are an obstacle」（11-06 实测）
        # 11-09：clear_entirely 会把整张代价地图（含静态地图层）重置成「未知」，等下一轮更新才填回；
        # Theta* 不许走未知区后，清完 1.5 s 规划常报起点/终点是障碍。改用 clear_except（reset_distance 0 =
        # 全部清掉）：只清雷达障碍层（clearable_layers 默认含 voxel_layer），静态地图不动
        self.clear_clis = [self.create_client(ClearCostmapExceptRegion, f'/{c}/clear_except_{c}')
                           for c in ('global_costmap', 'local_costmap')]

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
        self.map_file = None
        occ = (grid >= 65) & (grid != 255)
        occ = occ.reshape(info.height, info.width)
        near = occ.copy()
        for _ in range(max(1, round(MATCH_TOL / info.resolution))):   # 4 邻域膨胀
            n2 = near.copy()
            n2[1:, :] |= near[:-1, :]; n2[:-1, :] |= near[1:, :]; n2[:, 1:] |= near[:, :-1]; n2[:, :-1] |= near[:, 1:]
            near = n2
        self.near_occ, self.map_info = near, (info.resolution, info.origin.position.x, info.origin.position.y, info.width, info.height)
        if abs(yaw_of(info.origin.orientation)) < 1e-6:   # 自动定位只支持不旋转的地图原点（slam_toolbox / map_server 都是）
            self.map_raw = (np.asarray(m.data, dtype=np.int8).reshape(info.height, info.width),
                            info.resolution, info.origin.position.x, info.origin.position.y)
        else:
            self.map_raw = None
        self.map_ver += 1
        self.get_logger().info(f'map {info.width}x{info.height} @ {info.resolution:.3f} m')

    def show_file_map(self, name, m):
        """没有 map_server / slam_toolbox 时，直接显示 data/maps 里的地图（mapfile.load 的结果）。
        只给网页看、选点编辑路线；定位相关的东西（匹配率、自动定位用的栅格）只认 /map。"""
        grid = m['grid'].astype(np.int16)
        grid[grid < 0] = 255
        h, w = grid.shape
        meta = {'w': w, 'h': h, 'res': m['res'], 'ox': m['ox'], 'oy': m['oy'], 'oyaw': m['oyaw'], 'frame': 'map'}
        self.map_blob = pack(T_MAP, meta, zlib.compress(grid.astype(np.uint8).tobytes(), 6))
        self.map_file = name
        self.map_ver += 1
        self.get_logger().info(f'map {name} {w}x{h} @ {m["res"]:.3f} m（文件，导航没开）')

    def on_scan(self, m):
        self.rates['/scan'].tick()
        self.keep_scan_base(m)
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

    def keep_scan_base(self, m):
        """自动定位用：把 scan 转到 base_footprint 存下来（和地图坐标系无关，定位错了也能用）。"""
        if m.header.frame_id in ('', BASE_FRAME):
            tx = ty = th = 0.0
        else:
            try:
                tr = self.tf.lookup_transform(BASE_FRAME, m.header.frame_id, rclpy.time.Time()).transform
            except Exception:
                return
            tx, ty, th = tr.translation.x, tr.translation.y, yaw_of(tr.rotation)
        r = np.asarray(m.ranges, dtype=np.float32)
        a = m.angle_min + np.arange(r.size, dtype=np.float32) * m.angle_increment + th
        ok = np.isfinite(r) & (r >= max(m.range_min, 0.45)) & (r <= min(m.range_max, 20.0))
        r, a = r[ok], a[ok]
        # 只叠加静止时的帧：原地转动时几帧叠在一起会糊掉（11-10 用 bag 测到过）
        v = self.odom or (0.0, 0.0, 0.0)
        still = math.hypot(v[0], v[1]) < AUTOLOC_STILL[0] and abs(v[2]) < AUTOLOC_STILL[1]
        self.scans_base.append((time.monotonic(), np.stack([tx + r * np.cos(a), ty + r * np.sin(a)], 1), still))

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
            self.map_current = self.map_loaded = None
            return
        req = GetParameters.Request()
        req.names = ['yaml_filename']

        def done(fut):
            try:
                v = fut.result().values[0].string_value
                param = os.path.basename(v)[:-5] if v.endswith('.yaml') else (v or None)
                self.map_current = self.match_map(param)
            except Exception:
                pass
        self.mapparam_cli.call_async(req).add_done_callback(done)

    def match_map(self, param):
        """现在用的是哪张地图。map_server 的 yaml_filename 参数只反映启动时的地图：
        用 load_map 服务（网页「切换地图」）换图后它不会变（11-06 实测），所以以实际发出的 /map 为准，
        和 data/maps 里的文件比尺寸、原点、分辨率。内容一样分不出来时，优先网页加载的那张，再是参数。"""
        info = self.map_info
        if not info:
            return param
        res, ox, oy, w, h = info
        hits = [m['name'] for m in P.list_maps(os.path.join(DATA, 'maps'))
                if m['w'] == w and m['h'] == h and m['ox'] is not None
                and abs(m['res'] - res) < 1e-6 and abs(m['ox'] - ox) < 1e-3 and abs(m['oy'] - oy) < 1e-3]
        for name in (self.map_loaded, param):
            if name and name in hits:
                return name
        return hits[0] if len(hits) == 1 else param

    def on_odom(self, m):
        self.rates['/odom'].tick()
        t = m.twist.twist
        self.odom = (t.linear.x, t.linear.y, t.angular.z)
        p, st = m.pose.pose, m.header.stamp
        self.odom_pose = (p.position.x, p.position.y, yaw_of(p.orientation), time.monotonic())
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
    def publish_initial_pose(self, x, y, th, fine=False):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = FINE_COV_XY if fine else INIT_COV_XY
        cov[35] = FINE_COV_YAW if fine else INIT_COV_YAW
        m.pose.covariance = cov
        self.initpose_pub.publish(m)

    def stamped_pose(self, x, y, th):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        return p

    def send_goal(self, x, y, th):
        g = NavigateToPose.Goal()
        g.pose = self.stamped_pose(x, y, th)
        return self.nav.send_goal_async(g, feedback_callback=self.on_feedback)

    def compute_path(self, start, goal):
        g = ComputePathToPose.Goal()
        g.start = self.stamped_pose(start['x'], start['y'], start['th'])
        g.goal = self.stamped_pose(goal['x'], goal['y'], goal['th'])
        g.use_start = True
        g.planner_id = 'GridBased'
        return self.planner.send_goal_async(g)

    def track_goal(self, handle, pose):
        gid = bytes(handle.goal_id.uuid)
        with self.lock:
            self.goal = {'id': gid, 'pose': pose, 't0': time.monotonic(), 'status': 'active', 'end': None,
                         'remaining': None, 'total': None, 'nav_time': None, 'eta': None, 'recoveries': 0}
        result = handle.get_result_async()
        result.add_done_callback(lambda f: self.on_result(gid, f))
        return result

    def cancel_all(self):
        """取消所有 navigate_to_pose 目标（零 uuid + 零时间戳 = 全部），包括 RViz、patrol 发的。"""
        return self.cancel_cli.call_async(CancelGoal.Request())

    def clear_costmaps(self):
        """清空全局、局部代价地图（障碍层清空后按当前点云重新标记），返回各个服务的 future。"""
        return [c.call_async(ClearCostmapExceptRegion.Request(reset_distance=0.0))
                for c in self.clear_clis if c.service_is_ready()]

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
        if foreign:
            # 别处（RViz、命令行的 patrol.py）发的目标没有我们自己的 action 句柄，进度来自 /navigate_to_pose/_action/feedback
            fb = self.fb_any.get(foreign[0])
            fresh = fb and time.monotonic() - fb[4] < 3.0
            return {'type': 'external', 'status': 'active', 'phase': phase,
                    'remaining': fb[0] if fresh else None, 'elapsed': fb[1] if fresh else None,
                    'recoveries': fb[2] if fresh else 0, 'eta': fb[3] if fresh else None, 'total': None}
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
        # 网页不再启动巡逻；命令行（nav.sh）启动的 patrol.py 仍要识别：取消 / 急停 / 切档位时连它一起停
        profiles['patrol'] = 'run' if 'go2w_patrol' in names or (procs_status or {}).get('patrol') else 'stop'
        profiles['bag'] = 'run' if any(n.startswith('rosbag2_recorder') for n in names) or (procs_status or {}).get('bag') else 'stop'
        # 遥控用的独立速度桥（不开导航时）：进程在 + 节点在才算能用
        profiles['bridge'] = 'run' if (procs_status or {}).get('bridge') and 'go2w_cmd_bridge' in names else 'stop'
        for k, v in (transitions or {}).items():   # 网页刚发起的启停，进程状态还没跟上
            profiles[k] = v
        return profiles


class Server:
    def __init__(self, node, static_dir, heartbeat):
        self.node, self.static_dir, self.heartbeat = node, static_dir, heartbeat
        self.clients = {}       # ws → {'remote', 'last_ping'}
        self.owner = None       # 控制方：最近一个发过运动相关指令的网页
        self.estop, self.estop_t = False, 0.0
        self.pending = set()
        node.wants = self.wants   # 点云、粒子没人看就不算，省扩展坞的 CPU
        self.camera = C.Camera(pack, T_CAM, node.event)
        self.sys = SysStats()
        # P3：进程管理。本地调试（GO2W_FAKE_PROCS=1）换成假模拟器里的进程
        self.simulated = bool(os.environ.get('GO2W_FAKE_PROCS'))
        if self.simulated:
            from go2w_fake_procs import FakeProcs
            self.procs = FakeProcs(node)
        else:
            self.procs = P.DockerProcs()
        self.procs_status = None      # 最近一次进程检查结果；None = 查不到（没有 docker.sock）
        self.transitions = {}         # 网页发起、还没完成的启停：档位 → 'starting' / 'stopping'
        self.starting = {}            # 档位 → 开始启动的时间（启动超时 / 失败检测）
        self.bridge_t, self.bridge_started = -1e9, False   # 遥控用独立速度桥：上次启动时间、是否由网页启动过
        self.busy = False             # 同一时间只做一个启停操作
        self.map_choice = None        # 没有定位 / 导航在跑时，网页选好的「下次启动用的地图」
        # 最近用过 / 选过的地图（data/last_map.json）：导航没开时网页显示它，重启后也记得
        self.last_map_file = os.path.join(DATA, 'last_map.json')
        self.last_map = None
        try:
            with open(self.last_map_file, encoding='utf-8') as f:
                v = json.load(f).get('name')
            self.last_map = v if isinstance(v, str) and P.SAFE_NAME.fullmatch(v) else None
        except (OSError, ValueError, AttributeError):
            pass
        self.preview_key = None       # 已经显示的文件地图 (名字, 修改时间)；导航 / 建图在跑时为 None
        self.map_busy = False         # 保存和导出地图期间保护源快照及目标文件
        self.teleop = None            # (vx, vy, wz, 收到时间)；None = 没有在遥控
        self.maps = P.list_maps(os.path.join(DATA, 'maps'))
        self.bags = P.list_bags(os.path.join(DATA, 'bags'))
        self.rollback_file = os.path.join(DATA, 'rollback.json')
        self.rollback_on, self.rollback_t, self.rollback_started, self.rollback_saving = True, 0.0, False, False
        self.stuck, self.stuck_t = StuckWatch(), -STUCK_COOLDOWN
        self.quota_warned = False     # 超限又删不动时只报一次警，回到上限以下再复位
        try:
            with open(self.rollback_file) as f:
                self.rollback_on = bool(json.load(f).get('on', True))
        except (OSError, ValueError, AttributeError):
            pass
        self.uploads_file = os.path.join(DATA, 'bag_uploads.json')
        self.uploads = {}             # 包名 → {state: run/done/fail, pct, msg}；done 的存进 bag_uploads.json
        try:
            with open(self.uploads_file) as f:
                self.uploads = {k: {'state': 'done', 'pct': 100, 'msg': v} for k, v in json.load(f).items()}
        except (OSError, ValueError, AttributeError):
            pass
        self.routes_file = os.path.join(DATA, 'routes.json')
        self.routes = {}
        self.routes_error = False
        self.motion_stops = 0
        try:
            with open(self.routes_file, encoding='utf-8') as f:
                self.routes = {r['id']: self.route_record(r, r['id']) for r in json.load(f)}
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError, KeyError) as e:
            self.routes_error = True
            node.event('ERROR', f'无法读取导航任务 {self.routes_file}：{e}')
        self.route = None
        self.route_job = None
        self.goal_pending = False
        self.views_file = os.path.join(DATA, 'map_views.json')
        self.map_views = {}
        try:
            with open(self.views_file, encoding='utf-8') as f:
                raw = json.load(f)
            self.map_views = {k: self.view_angle(v) for k, v in raw.items() if isinstance(k, str) and P.SAFE_NAME.fullmatch(k)}
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError, AttributeError) as e:
            node.event('WARN', f'无法读取地图显示方向 {self.views_file}，全部按 0° 显示：{e}')
        self.map_loading = False
        self.checking_route = False
        # 自动定位：最近一次的结果推给网页（state.autoLoc）；seq 每次结束 +1，网页据此知道「刚定位成功」
        self.autoloc = {'state': 'idle', 'seq': 0}
        self.autoloc_busy = False
        self.pose_set_t = 0.0   # 最近一次（手动或自动）设置初始位姿的时间；启动后自动定位前用户已经手动给了就不再覆盖
        # 运控：站起 / 趴下（robot_ctrl.py）
        self.ctrl = None
        if CTRL_ON:
            try:
                self.ctrl = R.CtrlLink(node, node.event, fake=self.simulated)
            except Exception as e:
                node.event('ERROR', f'无法连接运控状态（域 0）：{e!r}；站起 / 趴下不可用')
        self.ctrl_op = None        # 'up' / 'down'：正在站起 / 趴下
        self.ctrl_abort = False    # 站起过程中被要求趴下
        self.ctrl_msg = ''         # 最近一次站起 / 趴下的结果
        self.stood_seq = 0         # 从趴着站起来的次数：网页据此作废定位（站起来时雷达位姿变了，要重新定位）
        self.auto_lie = False      # 空闲自动趴下生效（网页站起来过或跑过任务；遥控器站起来的不管）
        self.stand_hold = False    # 网页点了「保持站立」：不自动趴下，直到下次出发或手动趴下
        self.idle_since = None     # 开始空闲的时间
        self.prep_hold_t = -1e9    # 网页自检最近一次说「我在自检」的时间

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
        self.motion_stops += 1
        try:
            n, errors = self.node, []
            self.stop_route(reason)
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
        finally:
            self.motion_stops -= 1

    # ---------- 指令 ----------
    def remote(self, ws):
        return self.clients.get(ws, {}).get('remote', '?')

    async def run_cmd(self, ws, d):
        try:
            ok, text, *data = await self.handle_cmd(ws, d)
        except Exception as e:
            self.node.get_logger().error(f'cmd {d.get("op")}: {e!r}')
            ok, text, data = False, f'后端出错：{e!r}', []
        if not ws.closed:
            reply = {'t': 'ack', 'id': d.get('id'), 'ok': ok, 'msg': text}
            if data:
                reply['data'] = data[0]
            await ws.send_str(json.dumps(reply))

    async def handle_cmd(self, ws, d):
        n, op = self.node, d.get('op')
        if self.map_busy and op in ('set_mode', 'load_map', 'save_map', 'align_map'):
            return False, '地图正在保存或对齐，请稍等'
        if ws.closed or ws not in self.clients:
            return False, '网页已断开'
        if self.checking_route and op in ('goal', 'initial_pose', 'route_start', 'auto_localize'):
            return False, '正在检查路线，请稍等'
        if self.route_running() and (op in ('goal', 'initial_pose', 'auto_localize')
                                     or (op == 'arm' and d.get('on'))):
            return False, '多点任务进行中：请先取消任务'
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
                fine = d.get('cov') == 'fine'
                n.publish_initial_pose(d['x'], d['y'], d['th'], fine)
                self.pose_set_t = time.monotonic()
                n.event('INFO', f"{'重定位微调' if fine else '初始位姿'} ({d['x']:.2f}, {d['y']:.2f}, {math.degrees(d['th']):.1f}°)（{self.remote(ws)}）")
                # 11-09：定位错误期间（启动时 AMCL 默认在地图原点）雷达标的障碍会落在错误位置，
                # 机器人脚下的盲区又擦不掉 → 规划报「起点在障碍物里」。改位姿后清一次代价地图
                asyncio.ensure_future(self.clear_after_pose())
                return True, '位置已修正' if fine else '初始位姿已发布'
            if op == 'auto_localize':
                return await self.auto_localize(self.remote(ws))
            if op == 'stand_up':
                return await self.stand_up(self.remote(ws))
            if op == 'lie_down':
                self.owner = ws
                return await self.lie_down(self.remote(ws))
            if op == 'stand_hold':
                on = bool(d.get('on'))
                self.stand_hold, self.idle_since = on, None
                if on:
                    n.event('INFO', f'保持站立：不会自动趴下，直到下次出发或手动趴下（{self.remote(ws)}）')
                return True, '保持站立，不会自动趴下' if on else '空闲后自动趴下'
            if op == 'prep_hold':    # 网页自检进行中：别当成空闲趴下
                self.prep_hold_t = time.monotonic()
                return True, ''
            if op == 'goal':
                if self.estop:
                    return False, '急停中：先解除急停'
                reason = self.ctrl_blocker()
                if reason:
                    return False, reason
                if self.prof('patrol') != 'stop':
                    return False, '命令行启动的 patrol.py 在运行：先取消导航（会连巡逻一起停）'
                if not n.nav.server_is_ready():
                    return False, 'Nav2 没有运行（navigate_to_pose 不可用）'
                if self.goal_pending:
                    return False, '正在发送导航目标'
                self.route = None
                # 结束的旧目标（卡片会保留 5 s）先清掉：等 Nav2 接受新目标时，卡片不能显示旧目标的「已取消 / 失败」
                with n.lock:
                    if n.goal and n.goal['status'] != 'active':
                        n.goal = None
                self.goal_pending = True
                try:
                    h = await self.aw(n.send_goal(d['x'], d['y'], d['th']), 3.0)
                    if not h.accepted:
                        return False, 'Nav2 拒绝了这个目标'
                    n.track_goal(h, (d['x'], d['y'], d['th']))
                finally:
                    self.goal_pending = False
                n.event('INFO', f"导航目标 ({d['x']:.2f}, {d['y']:.2f}, {math.degrees(d['th']):.0f}°)")
                return True, '目标已下发' if n.is_armed() else '目标已下发，底盘上锁中，解锁后才会运动'
            if op == 'cancel':
                self.motion_stops += 1
                try:
                    self.stop_route('手动取消')
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
                finally:
                    self.motion_stops -= 1
            if op == 'arm':
                on = bool(d.get('on'))
                if on and self.estop:
                    return False, '急停中：先解除急停'
                if not on and self.route_running():
                    errors = await self.disarm_and_cancel('底盘上锁')
                    return (False, '；'.join(errors)) if errors else (True, '底盘已上锁，任务已取消')
                if not n.arm_cli.service_is_ready():
                    return False, '速度桥没有运行（刚开机或刚切换档位时要等几秒）'
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
            if op in ('route_save', 'route_delete'):
                result = self.save_route(d) if op == 'route_save' else self.delete_route(d.get('route_id'))
                if result[0]:
                    await self.send_all(self.routes_message())
                return result
            if op == 'route_check':
                return await self.check_route(ws, d)
            if op == 'route_start':
                return await self.start_route_armed(ws, d)
            if op == 'bag_start':
                return await self.bag_start(d.get('name'), d.get('topics') or [])
            if op == 'bag_delete':
                return await self.bag_delete(d.get('name'), self.remote(ws))
            if op == 'rollback_set':
                return self.rollback_set(bool(d.get('on')), self.remote(ws))
            if op == 'rollback_save':
                return await self.rollback_save(self.remote(ws))
            if op == 'bag_upload':
                return await self.bag_upload(d.get('name'), self.remote(ws))
            if op == 'bag_stop':
                await self.bag_stop()
                n.event('INFO', '录包已停止')
                return True, '录包已停止'
            if op == 'map_view':
                return await self.set_view(d.get('map'), d.get('angle'))
            if op == 'save_map':
                return await self.save_map(d.get('name', ''))
            if op == 'align_map':
                return await self.align_map(d.get('source'), d.get('name'), d.get('angle'))
            if op == 'load_map':
                return await self.load_map(d.get('name', ''))
            return False, f'不支持的操作：{op}'
        except asyncio.TimeoutError:
            return False, '对方没有响应（超时）'
        except (KeyError, TypeError, ValueError) as e:
            return False, f'指令格式不对：{e!r}'

    # ---------- 运控：站起 / 趴下（robot_ctrl.py） ----------
    # 站起 = 1004（Passive → FixStand，插值 2 s）→ 等站稳 → 1002（FixStand → Velocity，策略接管）→ 等 ready。
    # 趴下 = 1005（直接卸力塌下）。网页站起来（或跑过任务）后，空闲 IDLE_LIE 秒就原地趴下。
    def ctrl_get(self):
        return self.ctrl.get(CTRL_STALE) if self.ctrl else (None, False, -1)

    def ctrl_blocker(self):
        """导航能不能让机器狗动：要在策略（Velocity）里。没接运控（GO2W_CTRL=0）时不拦。"""
        if self.ctrl is None:
            return None
        s, ready, _ = self.ctrl_get()
        if s is None:
            return '收不到运控状态（/go2w_ctrl/state）：go2w-ctrl.service 没在运行？'
        if s != 'Velocity' or not ready:
            return f'机器狗没有站起来（运控状态 {s}）'
        return None

    def moving(self):
        n = self.node
        v, p = n.odom, n.odom_pose
        if v is None or p is None or time.monotonic() - p[3] > STALE:
            return False   # 没有里程计（导航没开）：不知道，按静止算
        return abs(v[0]) > STILL_V or abs(v[1]) > STILL_V or abs(v[2]) > STILL_W

    async def ctrl_switch(self, api, src, dst, timeout, need_ready=True):
        """在 src（元组）里时发 api（need_ready：还要等 src 的 ready），直到 dst 且 ready。返回 None 或失败原因。"""
        t0, sent = time.monotonic(), -1e9
        while True:
            if self.ctrl_abort and api != R.API_STANDDOWN:
                return '被「趴下」打断'
            s, ready, _ = self.ctrl_get()
            if s is None:
                return '收不到运控状态'
            if s == dst:
                if ready:
                    return None
            elif s not in src:
                return f'运控状态变成了 {s}'
            elif (ready or not need_ready) and time.monotonic() - sent > CTRL_RESEND:
                self.ctrl.send(api)
                sent = time.monotonic()
            if time.monotonic() - t0 > timeout:
                return f'{timeout:.0f} s 内没有完成（运控状态 {s}{"" if ready else "，未就绪"}）'
            await asyncio.sleep(0.05)

    def ctrl_done(self, ok, msg):
        self.ctrl_msg = msg
        self.node.event('INFO' if ok else 'ERROR', msg)
        return ok, msg

    async def stand_up(self, who):
        if self.ctrl is None:
            return False, '运控接入已关闭（GO2W_CTRL=0）'
        if self.ctrl_op:
            return False, '正在站起 / 趴下，请稍等'
        if self.estop:
            return False, '急停中：先解除急停'
        s, ready, _ = self.ctrl_get()
        if s is None:
            return False, '收不到运控状态（/go2w_ctrl/state）：go2w-ctrl.service 没在运行？'
        self.auto_lie, self.stand_hold, self.idle_since = True, False, None   # 网页站起来的：空闲了自动趴下
        if s == 'Velocity' and ready:
            return True, '已经站好（策略运行中）'
        if s not in R.STATES:
            return False, f'运控状态 {s}，不能站起'
        self.ctrl_op, self.ctrl_abort, lying = 'up', False, s == 'Passive'
        try:
            n = self.node
            if lying:
                n.event('INFO', f'站起：Passive → FixStand（{who}）')
                err = await self.ctrl_switch(R.API_STANDUP, ('Passive',), 'FixStand', CTRL_INTERP + CTRL_WAIT)
                if err:
                    return self.ctrl_done(False, f'站起失败：{err}')
            if self.estop:
                return self.ctrl_done(False, '急停中：停在站立姿态，没有启动策略')
            s, ready, _ = self.ctrl_get()
            if s != 'Velocity':
                n.event('INFO', '启动策略：FixStand → Velocity')
                # FixStand 没站稳时运控也接受 1002，会在半站的姿态下切进策略：ctrl_switch 等 ready 才发
                err = await self.ctrl_switch(R.API_BALANCESTAND, ('FixStand',), 'Velocity',
                                             CTRL_WAIT + (0 if ready else CTRL_INTERP))
                if err:
                    return self.ctrl_done(False, f'启动策略失败：{err}')
            if lying:
                self.stood_seq += 1
            return self.ctrl_done(True, '已站起，策略已接管')
        finally:
            self.ctrl_op = None

    async def lie_down(self, who, auto=False):
        """原地趴下。手动的先取消任务、上锁，等停稳；自动的只在空闲时触发（idle_tick）。"""
        if self.ctrl is None:
            return False, '运控接入已关闭（GO2W_CTRL=0）'
        s, _, _ = self.ctrl_get()
        if s is None:
            return False, '收不到运控状态（/go2w_ctrl/state）：go2w-ctrl.service 没在运行？'
        if s == 'Passive':
            self.auto_lie = self.stand_hold = False
            return True, '已经趴着'
        if self.ctrl_op == 'down':
            return False, '正在趴下'
        self.ctrl_abort = True                 # 正在站起就打断
        while self.ctrl_op == 'up':
            await asyncio.sleep(0.05)
        self.ctrl_op = 'down'
        try:
            n = self.node
            if (self.route_running() or n.nav_active() or self.teleop is not None or n.is_armed()
                    or self.prof('patrol') != 'stop'):
                await self.disarm_and_cancel(f'趴下（{who}）')
                if self.prof('patrol') != 'stop':
                    await self.procs.stop_patrol()
            t0 = time.monotonic()
            while self.moving() and time.monotonic() - t0 < 3.0:   # 走着的时候塌下去太危险：等停稳
                await asyncio.sleep(0.1)
            s, _, _ = self.ctrl_get()
            n.event('WARN', f'趴下（{who}）：{s} → Passive')
            err = await self.ctrl_switch(R.API_STANDDOWN, ('FixStand', 'Velocity'), 'Passive', CTRL_WAIT, need_ready=False)
            if err:
                return self.ctrl_done(False, f'趴下失败：{err}')
            self.auto_lie = self.stand_hold = False
            self.idle_since = None
            return self.ctrl_done(True, '已趴下' + ('（空闲自动）' if auto else ''))
        finally:
            self.ctrl_op, self.ctrl_abort = None, False

    def ctrl_busy(self):
        """有事在做，不算空闲：任务、导航、遥控、巡逻、检查、自动定位、站起趴下、档位切换、网页自检中、保持站立、还在走。"""
        n = self.node
        return bool(self.route_running() or self.goal_pending or n.nav_active() or self.teleop is not None
                    or self.checking_route or self.autoloc_busy or self.ctrl_op or self.busy or self.map_loading
                    or self.stand_hold or time.monotonic() - self.prep_hold_t < PREP_HOLD_TTL
                    or self.prof('patrol') != 'stop' or self.moving())

    def idle_tick(self):
        if self.ctrl is None or IDLE_LIE <= 0:
            return
        s, _, _ = self.ctrl_get()
        if s == 'Passive':
            self.auto_lie = self.stand_hold = False   # 已经趴下了（不管是谁让它趴的）：下次站起来再说
        if not self.auto_lie or s not in ('FixStand', 'Velocity') or self.ctrl_busy():
            self.idle_since = None
            return
        now = time.monotonic()
        if self.idle_since is None:
            self.idle_since = now
        elif now - self.idle_since >= IDLE_LIE:
            self.idle_since = None
            self.background(self.lie_down(f'空闲 {IDLE_LIE:.0f} s 自动', auto=True))

    def ctrl_view(self):
        if self.ctrl is None:
            return {'on': False}
        s, ready, seq = self.ctrl_get()
        left = None if self.idle_since is None else max(0.0, IDLE_LIE - (time.monotonic() - self.idle_since))
        return {'on': True, 'state': s, 'ready': ready, 'seq': seq, 'op': self.ctrl_op, 'msg': self.ctrl_msg,
                'stood': self.stood_seq, 'auto': self.auto_lie and IDLE_LIE > 0, 'hold': self.stand_hold,
                'lieIn': None if left is None else round(left, 1)}

    # ---------- 多点导航 ----------
    @staticmethod
    def route_next(n, loops, state):
        """循环路线的下一站：state = (站序号, 第几圈, 是否在回起点)，返回 None 表示走完。
        单程（loops = None）：0 → n-1。循环：0 → … → n-1 → 0 算一圈（终点直接回起点，不原路返回），
        下一圈从 1 出发；loops = 0 表示一直循环。"""
        i, loop, back = state
        if back:                          # 刚回到起点：这一圈走完
            return (1, loop + 1, False) if loops == 0 or loop < loops else None
        if i < n - 1:
            return i + 1, loop, False
        return (0, loop, True) if loops is not None else None

    @staticmethod
    def route_pose(points, i, back):
        """回到起点时也按 1 号点自己的朝向停（自动朝向 = 面向 2 号点），下一圈出发前不用再转。"""
        return points[i]

    @staticmethod
    def route_record(d, route_id):
        name, map_name, points = d['name'], d['map'], d['points']
        if not isinstance(name, str) or not 1 <= len(name.strip()) <= 60:
            raise ValueError('任务名称需要 1–60 个字符')
        if not isinstance(map_name, str) or not map_name:
            raise ValueError('需要绑定地图')
        if not isinstance(points, list) or not 2 <= len(points) <= 50:
            raise ValueError('每条路线需要 2–50 个点')
        clean = []
        for point in points:
            if not isinstance(point, dict) or any(
                type(point.get(k)) not in (float, int) or not math.isfinite(point[k]) for k in ('x', 'y', 'th')
            ):
                raise ValueError('路线包含无效坐标')
            label, manual = point.get('name', ''), point.get('manualHeading', True)
            if not isinstance(label, str) or len(label.strip()) > 40:
                raise ValueError('站点名称最多 40 个字符')
            if type(manual) is not bool:
                raise ValueError('站点朝向类型无效')
            clean.append({'x': point['x'], 'y': point['y'], 'th': point['th'],
                          'name': label.strip(), 'manualHeading': manual})
        return {'id': route_id, 'name': name.strip(), 'map': map_name, 'points': clean}

    def save_route(self, d):
        if self.routes_error:
            return False, '任务文件读取失败，请检查后端日志'
        shown = self.shown_map()
        if not shown or d.get('map') != shown or self.map_loading or self.busy:
            return False, '请在路线对应的地图上保存'
        route_id = d.get('route_id')
        if route_id is not None and route_id not in self.routes:
            return False, '找不到要更新的任务'
        route_id = route_id or uuid.uuid4().hex
        record = self.route_record(d, route_id)
        self.write_routes({**self.routes, route_id: record})
        return True, f"已保存任务：{record['name']}", record

    def delete_route(self, route_id):
        if self.routes_error:
            return False, '任务文件读取失败，请检查后端日志'
        if route_id not in self.routes:
            return False, '找不到要删除的任务'
        if self.route_running() and self.route['record']['id'] == route_id:
            return False, '请先结束正在执行的任务'
        routes = dict(self.routes)
        record = routes.pop(route_id)
        self.write_routes(routes)
        return True, f"已删除任务：{record['name']}"

    def write_routes(self, routes):
        os.makedirs(DATA, exist_ok=True)
        tmp = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=DATA, delete=False) as f:
                tmp = f.name
                json.dump(list(routes.values()), f, ensure_ascii=False, indent=2)
            os.replace(tmp, self.routes_file)
        finally:
            if tmp and os.path.exists(tmp):
                os.unlink(tmp)
        self.routes = routes

    @staticmethod
    def view_angle(v):
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError('角度必须是有限的数字')
        a = math.remainder(float(v), 2 * math.pi)     # 归到 [-π, π]
        return round(a, 6)

    def write_views(self, views):
        os.makedirs(DATA, exist_ok=True)
        tmp = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=DATA, delete=False) as f:
                tmp = f.name
                json.dump(views, f, ensure_ascii=False, indent=2, sort_keys=True)
            os.replace(tmp, self.views_file)
        finally:
            if tmp and os.path.exists(tmp):
                os.unlink(tmp)
        self.map_views = views

    def views_message(self):
        return json.dumps({'t': 'map_views', 'items': self.map_views})

    async def set_view(self, name, angle):
        if not isinstance(name, str) or not P.SAFE_NAME.fullmatch(name):
            return False, '地图名只能用字母、数字、下划线和减号'
        try:
            a = self.view_angle(angle)
        except ValueError as e:
            return False, str(e)
        views = dict(self.map_views)
        if abs(a) < 1e-6:
            views.pop(name, None)
        else:
            views[name] = a
        if views != self.map_views:
            try:
                self.write_views(views)
            except OSError as e:
                return False, f'保存显示方向失败：{e}'
            await self.send_all(self.views_message())
        return True, ''

    def routes_message(self):
        return json.dumps({'t': 'routes', 'items': list(self.routes.values())})

    def route_running(self):
        return self.route_job is not None and not self.route_job.done()

    def route_blocker(self, ws, route):
        n = self.node
        client = self.clients.get(ws)
        if client is None or ws.closed or time.monotonic() - client['last_ping'] > self.heartbeat:
            return '控制方网页连接已断开'
        if self.estop or not n.is_armed():
            return '急停或底盘未解锁'
        reason = self.ctrl_blocker()
        if reason:
            return reason
        if self.busy or self.map_loading or self.motion_stops or self.prof('navigation') != 'run' or not n.nav.server_is_ready():
            return '导航不可用'
        if not n.healthy or time.monotonic() - n.healthy_t >= STALE or not n.lookup(BASE_FRAME) or n.fixed != 'map':
            return '地图定位或里程计不可用'
        if n.map_current != route['map']:
            return f"请先切换到地图 {route['map']}"
        return None

    def cancel_late_goal(self, future):
        # 请求超时后仍可能被接受。只取消这个迟到的目标。
        try:
            handle = future.result()
            if handle.accepted:
                handle.cancel_goal_async()
        except Exception as e:
            self.node.event('ERROR', f'取消迟到的目标失败：{e}')

    async def plan_route(self, record, start=None, stopped=lambda: False):
        n, points = self.node, record['points']
        legs = list(zip(points, points[1:]))
        if start is not None:
            legs.insert(0, (start, points[0]))
        first = 1 if start is not None else 2
        deadline = time.monotonic() + 30.0
        paths = []
        for i, (a, b) in enumerate(legs, first):
            if stopped():
                return paths, i, '路线检查已中断'
            if self.busy or self.map_loading or n.map_current != record['map'] or not n.planner.server_is_ready():
                return paths, i, '当前地图的 Nav2 规划器不可用'
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return paths, i, '路线检查超时'
            handle = None
            request = n.compute_path(a, b)
            try:
                handle = await self.aw(request, min(2.0, remaining))
                if not handle.accepted:
                    return paths, i, 'Nav2 规划器拒绝了请求'
                reply = await self.aw(handle.get_result_async(), min(3.0, max(.01, deadline - time.monotonic())))
                if reply.status != GoalStatus.STATUS_SUCCEEDED or not reply.result.path.poses:
                    return paths, i, 'Nav2 找不到可行路径'
            except asyncio.TimeoutError:
                if handle is None:
                    request.add_done_callback(self.cancel_late_goal)
                else:
                    handle.cancel_goal_async()
                return paths, i, 'Nav2 规划超时'
            if stopped() or self.map_loading or self.busy or n.map_current != record['map']:
                return paths, i, '路线检查已中断'
            poses = reply.result.path.poses
            step = max(1, math.ceil(len(poses) / 180))
            sampled = poses[::step]
            if sampled[-1] is not poses[-1]:
                sampled.append(poses[-1])
            paths.append({'to': i, 'points': [[p.pose.position.x, p.pose.position.y] for p in sampled]})
        return paths, None, ''

    async def check_route(self, ws, d):
        record = self.route_record({**d, 'name': '规划检查'}, None)
        if self.checking_route or self.route_running() or self.goal_pending or self.node.nav_active() or self.teleop is not None or self.prof('patrol') != 'stop':
            return False, '请先结束当前导航、巡逻、遥控或路线检查'
        if self.prof('navigation') == 'stop':
            return False, '导航没有运行：出发前自检时会用 Nav2 检查路径'
        if self.busy or self.map_loading or self.node.map_current != record['map'] or not self.node.planner.server_is_ready():
            return False, '当前地图的 Nav2 规划器未就绪，路线尚未验证'
        self.checking_route = True
        try:
            paths, failed, reason = await self.plan_route(record, stopped=lambda: ws.closed or self.estop or self.node.nav_active())
            message = '全部站间路径可规划' if failed is None else f'第 {failed - 1} → {failed} 站：{reason}'
            return failed is None, message, {'paths': paths, 'failed': failed, 'simulated': self.simulated}
        finally:
            self.checking_route = False

    def start_route(self, ws, route_id, round_trip=False, loops=1):
        route = self.routes.get(route_id)
        if route is None:
            return False, '找不到这个导航任务'
        if type(round_trip) is not bool or type(loops) is not int or not 0 <= loops <= 1000:
            return False, '循环圈数需要是 0–1000 的整数（0 = 一直循环）'
        if (self.route_running() or self.goal_pending or self.node.nav_active()
                or self.prof('patrol') != 'stop' or self.teleop is not None):
            return False, '请先结束当前导航、巡逻或遥控'
        reason = self.route_blocker(ws, route)
        if reason:
            return False, reason
        self.owner = ws
        self.auto_lie, self.stand_hold = True, False   # 任务结束后空闲 IDLE_LIE 秒自动趴下
        n = len(route['points'])
        self.route = {'record': route, 'index': 0, 'status': 'active', 'started': time.monotonic(),
                      'message': '', 'stop': asyncio.Event(), 'goal': None, 'checking': True,
                      # 循环：loops 为 None 表示单程；done = 已到达的站数，用来算进度。一圈到 n 站（含回到起点）
                      'loops': loops if round_trip else None, 'loop': 1, 'back': False, 'done': 0,
                      'total': n if not round_trip else None if loops == 0 else 1 + loops * n,
                      'target': route['points'][0]}
        self.route_job = asyncio.create_task(self.run_route(ws, self.route))
        how = '' if not round_trip else '，一直循环' if loops == 0 else f'，循环 {loops} 圈'
        return True, f"开始任务：{route['name']}{how}"

    async def start_route_armed(self, ws, d):
        """网页「出发」：先把这次的速度上限（vmax / wmax，导航没开时在网页上选好的）设给 cmd_bridge，
        arm=true 时再解锁出发；出发失败就把刚解的锁再上回去。"""
        n, armed_here = self.node, False
        for key, top in (('vmax', 1.2), ('wmax', 1.5)):
            v = d.get(key)
            if v is None:
                continue
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not 0 < v <= top:
                return False, f'速度上限 {key} 需要在 0–{top} 之间'
            if n.params.get(key) is not None and abs(n.params[key] - v) < 1e-6:
                continue
            if not n.pset_cli.service_is_ready():
                return False, 'cmd_bridge 没有运行，无法设置速度上限'
            r = await self.aw(n.set_param(key, v), 2.0)
            res = r.results[0]
            if not res.successful:
                return False, f'速度上限没有生效：{res.reason}'
            n.params[key] = float(v)
            n.event('INFO', f'{BRIDGE} {PARAMS[key]} = {float(v):.2f}（出发前）')
        if d.get('arm') and not n.is_armed():
            if self.estop:
                return False, '急停中：先解除急停'
            if self.prof('navigation') != 'run' or not n.arm_cli.service_is_ready():
                return False, '导航没有运行（cmd_bridge 不可用），无法解锁'
            self.owner = ws          # 先认控制方：解锁后这个网页断线会自动上锁
            r = await self.aw(n.call_arm(True), 2.0)
            n.event('WARN' if r.success else 'ERROR', f"启动任务前自动解锁：{r.message}（{self.remote(ws)}）")
            if not r.success:
                return False, f'解锁失败：{r.message}'
            n.armed, n.armed_t, armed_here = True, time.monotonic(), True
        result = self.start_route(ws, d.get('route_id'), d.get('round_trip', False), d.get('loops', 1))
        if armed_here and not result[0]:
            try:
                r = await self.aw(n.call_arm(False), 2.0)
                if r.success:
                    n.armed, n.armed_t = False, time.monotonic()
            except Exception as e:
                n.event('ERROR', f'任务没有启动，重新上锁失败：{e}')
        return result

    def finish_route(self, route, status, message):
        route.update(status=status, message=message, ended=time.monotonic())
        route['stop'].set()
        self.node.event('ERROR' if status == 'aborted' else 'INFO', message)

    def stop_route(self, reason):
        if self.route and self.route['status'] == 'active':
            self.finish_route(self.route, 'canceled', reason)

    async def clear_after_pose(self):
        """设置初始位姿后清代价地图：先等 AMCL 用新位姿更新 map→odom，再清，重新标记时障碍就落在对的位置。"""
        n = self.node
        await asyncio.sleep(0.5)
        futs = n.clear_costmaps()
        if not futs:
            return
        try:
            await asyncio.gather(*(self.aw(f, 2.0) for f in futs))
            n.event('INFO', '位姿已更新，已清除代价地图里的旧障碍标记')
        except Exception as e:
            n.event('WARN', f'设置位姿后清除代价地图没有完成：{e!r}')

    # ---------- 自动定位 ----------
    async def auto_localize(self, who, on_start=False):
        """用最近几帧 scan 在整张地图上找机器人（globloc.py 子进程），可信就发给 AMCL。

        返回 (ok, 提示, 结果)。不可信（分数低 / 有歧义）时不发位姿，只报告，让人手动给。
        机器人必须静止；导航目标执行中拒绝（定位跳变会让正在跑的规划失效）。
        """
        n = self.node
        if self.autoloc_busy:
            return False, '自动定位正在进行'
        if 'amcl' not in n.get_node_names():
            return False, '定位（AMCL）没有运行，无法自动定位'
        if n.nav_active():
            return False, '导航进行中：先取消导航再自动定位'
        if n.map_raw is None:
            return False, '还没有收到地图（或地图原点带旋转，暂不支持）'
        self.autoloc_busy = True
        self.autoloc = {'state': 'run', 'seq': self.autoloc['seq'], 'map': n.map_current}
        path = None
        try:
            # 先等站稳、静止（11-12：站起来后立刻定位总报「机器人在动」，自检卡住）
            scans, why, w0 = None, '', time.monotonic()
            while time.monotonic() - w0 < AUTOLOC_SETTLE_WAIT:
                scans, why = self.autoloc_scans()
                if scans is not None:
                    break
                await asyncio.sleep(0.2)
            if scans is None:
                raise RuntimeError(f'{AUTOLOC_SETTLE_WAIT:.0f} s 内没等到机器人站稳静止（{why}）')
            if n.map_raw is None:
                raise RuntimeError('地图没有了')
            grid, res, ox, oy = n.map_raw
            map_name, map_ver = n.map_current, n.map_ver
            self.autoloc['map'] = map_name
            t0 = time.monotonic()
            fd, path = tempfile.mkstemp(prefix='go2w_autoloc_', suffix='.npz')
            os.close(fd)
            np.savez(path, grid=grid, res=res, origin=np.array([ox, oy]), pts=np.concatenate(scans).astype(np.float32))
            script = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'globloc.py')
            # nice：别和 FAST-LIO / AMCL 抢 CPU
            proc = await asyncio.create_subprocess_exec('nice', '-n', '10', 'python3', script, path,
                                                        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
            try:
                out, err = await asyncio.wait_for(proc.communicate(), AUTOLOC_TIMEOUT)
            except asyncio.TimeoutError:
                proc.kill()
                raise RuntimeError(f'{AUTOLOC_TIMEOUT:.0f} s 内没有算完')
            if proc.returncode != 0:
                lines = err.decode(errors='replace').strip().splitlines()
                raise RuntimeError(lines[-1] if lines else f'退出码 {proc.returncode}')
            r = json.loads(out.decode().strip().splitlines()[-1])
            if 'error' in r:
                raise RuntimeError(r['error'])
        except Exception as e:
            msg = f'自动定位失败：{e}'
            self.autoloc = {'state': 'fail', 'seq': self.autoloc['seq'] + 1, 'map': n.map_current, 'msg': msg}
            n.event('ERROR', msg)
            return False, msg
        finally:
            self.autoloc_busy = False
            if path:
                try:
                    os.unlink(path)
                except OSError:
                    pass
        took = time.monotonic() - t0
        map_name = n.map_current or map_name
        where = f"({r['x']:.2f}, {r['y']:.2f}, {math.degrees(r['th']):.0f}°)"
        quality = f"分数 {r['score']:.2f}，次优/最优 {r['ratio']:.2f}，{took:.1f} s"
        res_ = {k: r[k] for k in ('x', 'y', 'th', 'score', 'ratio', 'second_pose')}
        # 只看栅格本身：启动时地图名（map_current）比栅格晚几秒才拿到，map_server 也可能重发同一张图
        now_raw = n.map_raw
        if n.map_ver != map_ver and (now_raw is None or now_raw[1:] != (res, ox, oy) or not np.array_equal(now_raw[0], grid)):
            msg = '自动定位期间地图变了，结果作废'
            self.autoloc = {'state': 'fail', 'seq': self.autoloc['seq'] + 1, 'map': map_name, 'msg': msg}
            n.event('WARN', msg)
            return False, msg
        if on_start and self.pose_set_t > t0:   # 算的时候用户已经手动给了位置：以手动为准
            self.autoloc = {'state': 'idle', 'seq': self.autoloc['seq']}
            n.event('INFO', f'自动定位结果 {where} 没有使用：期间已手动设置了初始位姿')
            return False, '期间已手动设置初始位姿'
        if not r['ok']:
            why = '匹配分数太低（不在这张地图上，或环境变化太大）' if r['score'] < 0.75 else '有多个相似位置，分不清'
            msg = f'自动定位不可信，没有使用：{why}。最佳 {where}，{quality}。请手动给初始位置'
            self.autoloc = {'state': 'fail', 'seq': self.autoloc['seq'] + 1, 'map': map_name, 'msg': msg, **res_}
            n.event('WARN', msg)
            return False, msg, res_
        n.publish_initial_pose(r['x'], r['y'], r['th'], fine=True)
        self.pose_set_t = time.monotonic()
        msg = f'自动定位成功 {where}（{quality}）'
        self.autoloc = {'state': 'ok', 'seq': self.autoloc['seq'] + 1, 'map': map_name, 'msg': msg, **res_}
        n.event('INFO', f'{msg}（{who}）')
        asyncio.ensure_future(self.clear_after_pose())
        return True, msg, res_

    def autoloc_scans(self):
        """能用来自动定位的 scan：站稳 AUTOLOC_SETTLE 秒之后收到的、静止的、最近 2 s 内的，至少 3 帧。
        返回 (点列表, '') 或 (None, 还差什么)。"""
        n, now = self.node, time.monotonic()
        since = 0.0
        if self.ctrl is not None:
            s, ready, _ = self.ctrl_get()
            if self.ctrl_op:
                return None, '正在站起 / 趴下'
            if s in ('FixStand', 'Velocity'):   # 趴着（Passive）也让定位：重定位标签里可能就是趴着在看
                st = self.ctrl.steady_since()
                if not ready or st is None:
                    return None, '还没站稳'
                if now - st < AUTOLOC_SETTLE:
                    return None, '刚站起来，等机身稳定'
                since = st + AUTOLOC_SETTLE
        recent = [(p, still) for t, p, still in list(n.scans_base) if now - t < 2.0 and t >= since]
        if len(recent) < 3:
            return None, '激光数据不足'
        scans = [p for p, still in recent if still]
        if len(scans) < 3:
            return None, '机器人在动'
        return scans, ''

    async def auto_localize_on_start(self, mode):
        """定位 / 导航档位刚启动：等地图和 scan 到齐、机器人静止，自动定位一次。"""
        n, t0 = self.node, time.monotonic()
        await asyncio.sleep(2.0)   # AMCL 刚激活，scan 刚开始出
        while time.monotonic() - t0 < 30:
            if self.prof(mode) == 'stop' or self.pose_set_t > t0:
                return
            now = time.monotonic()
            fresh = sum(1 for t, _, still in list(n.scans_base) if now - t < 2.0 and still)
            if n.map_raw is not None and n.map_current and fresh >= 3 and 'amcl' in n.get_node_names():
                break
            await asyncio.sleep(1.0)
        else:
            n.event('WARN', '启动后 30 s 内没等到地图 / 激光，没有自动定位，请手动给初始位置')
            return
        n.event('INFO', f'{self.MODE_NAME[mode]}已启动，开始自动定位（机器人保持静止）')
        await self.auto_localize('启动后自动', on_start=True)

    async def clear_before_start(self):
        """出发前清代价地图，再等一下让障碍层按当前点云重新标记。清不了只记日志，不拦着出发。"""
        n = self.node
        futs = n.clear_costmaps()
        if not futs:
            n.event('WARN', '代价地图清除服务不可用，没有清除旧的障碍标记')
            return
        try:
            await asyncio.gather(*(self.aw(f, 2.0) for f in futs))
        except Exception as e:
            n.event('WARN', f'清除代价地图没有完成：{e!r}')
            return
        await asyncio.sleep(COSTMAP_REFILL)
        n.event('INFO', '出发前已清除代价地图里的旧障碍标记')

    async def run_route(self, ws, route):
        n, record = self.node, route['record']
        try:
            await self.clear_before_start()
            if route['stop'].is_set():
                return
            pos = n.lookup(BASE_FRAME)
            if not pos or n.fixed != 'map':
                raise RuntimeError('无法读取当前位置')
            start = dict(zip(('x', 'y', 'th'), pos))
            points, loops = record['points'], route['loops']
            first = (0, 1, False)
            near = math.hypot(start['x'] - points[0]['x'], start['y'] - points[0]['y'])
            if near < ROUTE_NEAR_START:
                first = self.route_next(len(points), loops, first)
                route['done'] = 1
                n.event('INFO', f'已在第 1 站附近（{near:.2f} m），直接前往第 2 站')
            # 出发前检查一整圈（循环时含终点回起点那一段，后面几圈走的是同样的路段）
            order, state = [], first
            while state and state[1] == 1:
                order.append(state)
                state = self.route_next(len(points), loops, state)
            check = {**record, 'points': [self.route_pose(points, i, back) for i, _, back in order]}
            _, failed, reason = await self.plan_route(check, start, lambda: route['stop'].is_set() or n.nav_active())
            if route['stop'].is_set():
                return
            if failed is not None:
                route['index'] = order[failed - 1][0]
                to = order[failed - 1][0] + 1
                leg = f'当前位置 → 第 {to} 站' if failed == 1 else f'第 {order[failed - 2][0] + 1} → {to} 站'
                if failed >= 2 and order[failed - 1][2]:
                    leg = '回到起点（' + leg + '）'
                self.finish_route(route, 'aborted', f'{leg}规划检查失败：{reason}')
                return
            route['checking'] = False
            state = first
            while state:
                i, loop, back = state
                if route['stop'].is_set():
                    return
                point = self.route_pose(points, i, back)
                route.update(index=i, loop=loop, back=back, target=point, goal=None)
                reason = self.route_blocker(ws, record)
                if reason:
                    raise RuntimeError(reason)
                pose = (point['x'], point['y'], point['th'])
                request = n.send_goal(*pose)
                try:
                    handle = await self.aw(request, 3.0)
                except asyncio.TimeoutError:
                    request.add_done_callback(self.cancel_late_goal)
                    raise RuntimeError('Nav2 接收目标超时') from None
                if not handle.accepted:
                    raise RuntimeError('Nav2 拒绝目标')
                if route['stop'].is_set():
                    await self.aw(handle.cancel_goal_async(), 2.0)
                    return
                result = n.track_goal(handle, pose)
                route['goal'] = dict(zip(('x', 'y', 'th'), pose))
                stop = asyncio.create_task(route['stop'].wait())
                done = asyncio.create_task(self.aw(result, 180.0))
                try:
                    await asyncio.wait((stop, done), return_when=asyncio.FIRST_COMPLETED)
                    if route['stop'].is_set():
                        return
                    status = done.result().status
                except asyncio.TimeoutError:
                    await self.aw(handle.cancel_goal_async(), 2.0)
                    raise RuntimeError('等待到点超时') from None
                finally:
                    stop.cancel()
                    done.cancel()
                    await asyncio.gather(stop, done, return_exceptions=True)
                if route['stop'].is_set():
                    return
                if status == GoalStatus.STATUS_CANCELED:
                    self.finish_route(route, 'canceled', f'第 {i + 1} 点已取消')
                    return
                if status != GoalStatus.STATUS_SUCCEEDED:
                    here = n.lookup(BASE_FRAME)
                    off = math.hypot(here[0] - pose[0], here[1] - pose[1]) if here and n.fixed == 'map' else None
                    if off is None or off > ROUTE_ARRIVE_TOL:
                        raise RuntimeError('Nav2 未能到达' + (f'（离目标 {off:.2f} m）' if off is not None else ''))
                    n.event('WARN', f'第 {i + 1} 站未精确到达（偏差 {off:.2f} m），视为已到，继续下一站')
                route['done'] += 1
                state = self.route_next(len(points), loops, state)
            self.finish_route(route, 'succeeded', '已到达全部导航点' if loops is None else f'已完成循环 {loops} 圈')
        except Exception as e:
            if not route['stop'].is_set():
                self.finish_route(route, 'aborted', f'第 {route["index"] + 1} 点失败：{e}')
            else:
                n.event('ERROR', f'多点任务停止时出错：{e}')

    def task(self):
        goal, route = self.node.task(), self.route
        if not route:
            return goal
        status = route['status']
        # 结束的路线保留 10 s 给网页看结果；期间发了新的导航目标就立刻让位
        newer = goal and goal.get('status') == 'active' and goal.get('goal') != route['goal']
        if status != 'active' and (newer or time.monotonic() - route['ended'] > 10):
            self.route = None
            return goal
        record, i = route['record'], route['index']
        leg = goal if goal and goal['type'] == 'goal' and goal['goal'] == route['goal'] else {}
        phase = status
        if status == 'active':
            phase = 'checking' if route['checking'] else leg['phase'] if leg.get('status') == 'active' else 'plan'
        n, loops = len(record['points']), route['loops']
        if status == 'succeeded':
            progress = 1.0
        elif route['total']:
            progress = route['done'] / route['total']
        else:   # 一直循环：显示当前这一圈的进度
            progress = ((route['done'] - 1) % n) / n if route['done'] else 0.0
        return {**leg, 'type': 'route', 'name': record['name'], 'status': status, 'phase': phase,
                'goal': route['target'], 'point': i + 1, 'count': n,
                'roundTrip': loops is not None, 'loop': route['loop'], 'loops': loops, 'back': route['back'],
                'progress': progress,
                'routeId': record['id'], 'pointName': record['points'][i]['name'],
                'message': route['message'], 'elapsed': route.get('ended', time.monotonic()) - route['started']}

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
        if self.busy or self.map_busy:
            return False, '上一个启停还没完成，稍等几秒'
        if mode is not None and mode not in P.MODES:
            return False, f'没有这个档位：{mode}'
        cur = [m for m in P.MODES if self.prof(m) != 'stop']
        if mode is not None and cur == [mode]:
            return True, f'{self.MODE_NAME[mode]}已经在运行'
        if mode in ('localization', 'navigation'):
            names = [m['name'] for m in self.maps]
            map_name = map_name or self.default_map()
            if map_name not in names:
                return False, 'data/maps 里没有可用的地图'
        if not cur and mode is None:
            return True, '没有在运行的档位'
        self.busy = True
        self.stop_route('切换运行档位')
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
                ok = await self.procs.stop_modes()   # 会顺带杀掉遥控用的速度桥（同一个 pkill 模式）
                self.bridge_t, self.bridge_started = -1e9, False   # 不算异常退出；停完马上可以重新拉起
                await self.refresh_procs()
                for m in cur:
                    self.transitions.pop(m, None)
                    self.starting.pop(m, None)
                if ok:
                    n.event('INFO', f"已停止{'、'.join(self.MODE_NAME[m] for m in cur) or '巡逻'}")
                else:
                    n.event('ERROR', '还有导航进程没退出，没有继续启动新档位。可以再点一次，或在扩展坞上执行 nav.sh stop')
                    return
            if mode == 'navigation' and (self.procs_status or {}).get('bridge'):
                await self.stop_bridge('启动导航')
                if (self.procs_status or {}).get('bridge'):
                    n.event('ERROR', '遥控用的速度桥没有停掉，没有启动导航（两个速度桥不能同时跑）。可以再点一次')
                    return
            if mode:
                self.autoloc = {'state': 'idle', 'seq': self.autoloc['seq']}   # 上一轮的自动定位结果作废
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

    async def bag_start(self, name, topics):
        """分段录包：每 BAG_SEGMENT 秒一个包，包名 = 前缀_MMDD_HHMMSS。name 是前缀，不填用 web。"""
        name = name or 'web'
        if not P.SAFE_NAME.match(name) or len(name) > 30:
            return False, '包名前缀只能用字母、数字、下划线和减号，最多 30 个字符'
        bad = [t for t in topics if not P.SAFE_TOPIC.match(t)]
        if not topics or bad:
            return False, f"topic 写法不对：{' '.join(bad) or '（空）'}"
        if self.prof('bag') != 'stop':
            return False, '已经在录包了'
        over = await self.bag_quota()
        if over:
            return False, f'{over}：已上传的旧包删完了还是超限，先上传或手动删掉一些包再录'
        self.transitions['bag'] = 'starting'
        try:
            await self.procs.start_bag(name, topics)
            await asyncio.sleep(0.3)
            await self.refresh_procs()
        finally:
            self.transitions.pop('bag', None)
        self.node.event('INFO', f"开始录包（每 {P.BAG_SEGMENT} 秒一个包，{name}_时间）：{' '.join(topics)}")
        return True, f'开始录包：每 {P.BAG_SEGMENT} 秒存一个包到 data/bags/{name}_<时间>'

    async def bag_delete(self, name, who):
        # 删掉 data/bags/<name> 整个目录（.mcap + metadata.yaml），删了不能恢复
        if not name or not P.SAFE_NAME.match(name):
            return False, '包名不对'
        bags_dir = os.path.join(DATA, 'bags')
        self.bags = P.list_bags(bags_dir)
        if not any(b['name'] == name for b in self.bags):
            return False, f'data/bags 里没有 {name}'
        if self.prof('bag') != 'stop' or self.transitions.get('bag'):
            # 正在录的那个不能删：网页启动的记得名字；后端重启过或者不是网页启动的，保险起见最新的一个也不让删
            if name in self.recording_names():
                return False, f'{name} 正在录制，先停止录包再删'
        if self.uploads.get(name, {}).get('state') == 'run':
            return False, f'{name} 正在上传，等上传完再删'
        path = os.path.join(bags_dir, name)
        if os.path.realpath(os.path.dirname(path)) != os.path.realpath(bags_dir):
            return False, '包名不对'
        size = next(b['size'] for b in self.bags if b['name'] == name)
        try:
            await asyncio.to_thread(shutil.rmtree, path)
        except OSError as e:
            self.bags = P.list_bags(bags_dir)
            return False, f'删除失败：{e.strerror or e}'
        self.bags = P.list_bags(bags_dir)
        self.node.event('WARN', f'已删除录包 {name}（{size:.0f} MB，{who}）')
        return True, f'已删除录包 {name}'

    async def bag_stop(self):
        self.transitions['bag'] = 'stopping'
        try:
            await self.procs.stop_bag()
            await self.refresh_procs()
            self.bags = P.list_bags(os.path.join(DATA, 'bags'))
        finally:
            self.transitions.pop('bag', None)

    def bag_over(self, total_mb, free_mb):
        """超限就返回原因，没超返回空串。"""
        if total_mb > BAG_LIMIT_GB * 1024:
            return f'录包总量 {total_mb / 1024:.1f} GB 超过上限 {BAG_LIMIT_GB:.0f} GB'
        if free_mb is not None and free_mb < BAG_MIN_FREE_GB * 1024:
            return f'磁盘只剩 {free_mb / 1024:.1f} GB（低于 {BAG_MIN_FREE_GB:.0f} GB）'
        return ''

    async def bag_quota(self):
        """容量检查：超限就从最旧的开始删已上传的包。返回删完之后仍然超限的原因（没超返回空串）。"""
        bags_dir = os.path.join(DATA, 'bags')
        self.bags = P.list_bags(bags_dir)
        try:
            free = shutil.disk_usage(bags_dir).free / 1048576
        except OSError:
            free = None
        total = sum(b['size'] for b in self.bags)
        over = self.bag_over(total, free)
        if not over:
            self.quota_warned = False
            return ''
        recording = self.prof('bag') != 'stop' or bool(self.transitions.get('bag'))
        busy = self.recording_names() if recording else set()
        for b in reversed(self.bags):   # list_bags 是新的在前
            if not self.bag_over(total, free):
                break
            if b['name'] in busy or self.uploads.get(b['name'], {}).get('state') != 'done':
                continue
            ok, _ = await self.bag_delete(b['name'], f'自动清理，{over}')
            if ok:
                total -= b['size']
                free = free + b['size'] if free is not None else None
        over = self.bag_over(total, free)
        if not over:
            self.quota_warned = False
            return ''
        if recording and not self.transitions.get('bag'):
            cur = next((b for b in self.bags if b['name'] in busy), None)
            disk_low = free is not None and free < BAG_MIN_FREE_GB * 1024
            if disk_low or (cur and cur['size'] > BAG_LIMIT_GB * 1024):
                which = ' ' + cur['name'] if cur else ''
                self.node.event('ERROR', f'{over}，自动停止录包{which}')
                await self.bag_stop()
                self.quota_warned = True
                return over
        if not self.quota_warned:
            self.quota_warned = True
            self.node.event('WARN', f'{over}，已上传的旧包删完了还是超限：没上传的包不会自动删，请上传或手动删除')
        return over

    def recording_names(self):
        """列表里正在写的包。网页录包是分段的：正在写的那段在 .recording 里，列表里的都已经写完，返回空。
        命令行手动跑的 ros2 bag record 不知道名字，保险起见把最新的一个（回溯包除外）当成正在录。"""
        if (self.procs_status or {}).get('bag_seg'):
            return set()
        newest = next((b['name'] for b in self.bags if not b['name'].startswith(('snap_', 'stuck_'))), None)
        return {newest}

    def rollback_set(self, on, who):
        self.rollback_on = on
        try:
            with open(self.rollback_file, 'w') as f:
                json.dump({'on': on}, f)
        except OSError:
            pass
        self.node.event('INFO', f"回溯录制已{'打开' if on else '关闭'}（{who}）")
        return True, f"回溯录制已{'打开' if on else '关闭'}"

    async def stop_bridge(self, why):
        n = self.node
        if self.teleop is not None:
            self.teleop = None
        if n.is_armed():
            await self.disarm_and_cancel(why)
        await self.procs.stop_bridge()
        await self.refresh_procs()

    async def bridge_tick(self):
        """没开导航时，保持一个遥控用的速度桥在跑；导航启动 / 运行时不跑（导航自带速度桥）。"""
        st, n = self.procs_status, self.node
        if st is None or self.busy:
            return
        nav = st.get('navigation') or 'navigation' in self.starting or self.transitions.get('navigation')
        now = time.monotonic()
        if not nav and not st.get('bridge') and now - self.bridge_t > BRIDGE_RETRY:
            if self.bridge_started:
                n.event('WARN', '遥控用的速度桥退出了，正在重新启动。data/logs/teleop_bridge.log 最后几行：\n'
                        + P.tail(os.path.join(DATA, 'logs', 'teleop_bridge.log'), 6))
            self.bridge_t, self.bridge_started = now, True
            await self.procs.start_bridge()
        elif nav and st.get('bridge'):
            self.bridge_started = False
            await self.stop_bridge('导航启动')

    async def rollback_tick(self):
        """建图 / 定位 / 导航运行时保持回溯进程在跑，都停了就停掉它。"""
        st, n = self.procs_status, self.node
        if st is None:
            return
        want = self.rollback_on and any(st.get(m) for m in P.MODES)
        now = time.monotonic()
        if want and not st.get('rollback') and now - self.rollback_t > ROLLBACK_RETRY:
            if self.rollback_started:
                n.event('WARN', '回溯录制进程退出了，正在重新启动。data/logs/rollback.log 最后几行：\n'
                        + P.tail(os.path.join(DATA, 'logs', 'rollback.log'), 6))
            self.rollback_t, self.rollback_started = now, True
            await self.procs.start_rollback(ROLLBACK_SECONDS)
        elif st.get('rollback') and not want:
            self.rollback_started = False
            await self.procs.stop_rollback()

    def stuck_tick(self):
        """导航目标没到、狗却长时间停着不动时，自动保存一段回溯包。"""
        n, now = self.node, time.monotonic()
        o = n.odom_pose
        pose = o[:3] if o and now - o[3] < STALE else None
        active = n.nav_active() and bool(n.is_armed()) and not self.estop
        still = self.stuck.update(now, pose, active)
        if still is None:
            return
        if not (self.procs_status or {}).get('rollback'):
            n.event('WARN', f'导航中已经 {still:.0f} 秒没动了，但回溯录制没在运行，没法自动存包')
            return
        if now - self.stuck_t < STUCK_COOLDOWN:
            n.event('WARN', f'导航中已经 {still:.0f} 秒没动了（{STUCK_COOLDOWN:.0f} 秒内已自动存过包，这次不存）')
            return
        self.stuck_t = now
        n.event('WARN', f'导航中已经 {still:.0f} 秒没动了，还没到目标，{STUCK_DELAY:.0f} 秒后自动保存回溯包')

        async def later():
            await asyncio.sleep(STUCK_DELAY)
            for _ in range(10):   # 正好有人在手动保存时等它存完
                if not self.rollback_saving:
                    break
                await asyncio.sleep(1.0)
            await self.rollback_save('卡住自动保存', 'stuck_')
        self.background(later())

    async def rollback_save(self, who, prefix='snap_'):
        if not (self.procs_status or {}).get('rollback'):
            return False, '回溯录制没在运行：先打开回溯，并启动建图 / 定位 / 导航'
        if self.rollback_saving:
            return False, '正在保存上一段，请稍等'
        self.rollback_saving = True
        name, bags_dir = time.strftime(f'{prefix}%m%d_%H%M%S'), os.path.join(DATA, 'bags')
        work = os.path.join(bags_dir, '.rollback')
        error, done = os.path.join(work, f'{name}.error'), os.path.join(bags_dir, name, 'metadata.yaml')
        try:
            os.makedirs(work, exist_ok=True)
            tmp = os.path.join(work, 'request.tmp')
            with open(tmp, 'w') as f:
                f.write(name)
            os.replace(tmp, os.path.join(work, 'request'))
            t0 = time.monotonic()
            while time.monotonic() - t0 < ROLLBACK_SAVE_TIMEOUT:
                if os.path.exists(error):
                    with open(error, errors='replace') as f:
                        why = f.read()[:300]
                    os.remove(error)
                    self.node.event('ERROR', f'回溯保存失败：{why}')
                    return False, f'回溯保存失败：{why}'
                if os.path.exists(done):
                    self.bags = P.list_bags(bags_dir)
                    b = next((x for x in self.bags if x['name'] == name), {})
                    info = f"{b.get('dur') or 0:.0f} 秒，{b.get('size', 0):.0f} MB"
                    self.node.event('INFO', f'已保存回溯 {name}（{info}，{who}）')
                    return True, f'已保存回溯 {name}（{info}）'
                await asyncio.sleep(0.1)
            self.node.event('ERROR', f'回溯保存超时（{ROLLBACK_SAVE_TIMEOUT:.0f} s），看 data/logs/rollback.log')
            return False, '回溯保存超时，看 data/logs/rollback.log'
        finally:
            self.rollback_saving = False

    async def bag_upload(self, name, who):
        """rsync 把 data/bags/<name> 传到 166:~/go2w_bags/<name>。同一时间只传一个；进度和结果进网页日志。"""
        if not name or not P.SAFE_NAME.match(name):
            return False, '包名不对'
        bags_dir = os.path.join(DATA, 'bags')
        self.bags = P.list_bags(bags_dir)
        bag = next((b for b in self.bags if b['name'] == name), None)
        if bag is None:
            return False, f'data/bags 里没有 {name}'
        if self.prof('bag') != 'stop' or self.transitions.get('bag'):
            if name in self.recording_names():
                return False, f'{name} 正在录制，先停止录包再上传'
        busy = next((k for k, u in self.uploads.items() if u['state'] == 'run'), None)
        if busy:
            return False, f'正在上传 {busy}，等它传完'
        key = os.path.join(DATA, 'ssh', 'id_ed25519')
        if not os.path.exists(key) or not shutil.which('rsync') or not shutil.which('ssh'):
            return False, '扩展坞上缺上传工具：需要 data/ssh 密钥，以及镜像里的 rsync / ssh（重建 go2w-humble 镜像）'
        self.uploads[name] = {'state': 'run', 'pct': 0, 'msg': '连接中'}
        asyncio.create_task(self.bag_upload_run(name, bag['size'], key, who))
        return True, f'开始上传 {name} → {UPLOAD_LABEL}'

    async def bag_upload_run(self, name, size, key, who):
        n, t0 = self.node, time.monotonic()
        ssh = (f'ssh -i {key} -o UserKnownHostsFile={os.path.join(DATA, "ssh", "known_hosts")} '
               '-o StrictHostKeyChecking=yes -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=10')
        n.event('INFO', f'开始上传录包 {name}（{size:.0f} MB）→ {UPLOAD_LABEL}（{who}）')
        tail, last_pct = deque(maxlen=6), 0
        try:
            p = await asyncio.create_subprocess_exec(
                'rsync', '-rt', '--partial', '--info=progress2', '--no-inc-recursive', '-e', ssh,
                os.path.join(DATA, 'bags', name), UPLOAD_DEST,
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
            buf = b''
            while True:
                chunk = await p.stdout.read(4096)
                if not chunk:
                    break
                buf += chunk
                # progress2 用 \r 刷新同一行：「  12,345,678  45%   10.2MB/s    0:00:12」
                *lines, buf = buf.replace(b'\r', b'\n').split(b'\n')
                for raw in lines:
                    line = raw.decode('utf-8', 'replace').strip()
                    if not line:
                        continue
                    parts = line.split()
                    if len(parts) >= 3 and parts[1].endswith('%') and parts[1][:-1].isdigit():
                        pct = int(parts[1][:-1])
                        self.uploads[name] = {'state': 'run', 'pct': pct, 'msg': parts[2]}
                        if pct // 25 > last_pct // 25 and pct < 100:
                            n.event('INFO', f'上传 {name}：{pct}%（{parts[2]}）')
                        last_pct = max(last_pct, pct)
                    else:
                        tail.append(line)
            code = await p.wait()
        except Exception as e:
            code, tail = -1, deque([repr(e)])
        dt = time.monotonic() - t0
        if code == 0:
            msg = time.strftime('%m-%d %H:%M')
            self.uploads[name] = {'state': 'done', 'pct': 100, 'msg': msg}
            done = {k: u['msg'] for k, u in self.uploads.items() if u['state'] == 'done'}
            try:
                with open(self.uploads_file, 'w') as f:
                    json.dump(done, f, ensure_ascii=False, indent=1)
            except OSError:
                pass
            n.event('INFO', f'已上传录包 {name} → {UPLOAD_LABEL}/{name}（{size:.0f} MB，用时 {dt:.0f} s）')
        else:
            why = ' | '.join(tail) or f'rsync 退出码 {code}'
            self.uploads[name] = {'state': 'fail', 'pct': last_pct, 'msg': why[-200:]}
            n.event('ERROR', f'上传录包 {name} 失败（退出码 {code}，{dt:.0f} s）：{why[-300:]}')

    async def save_map(self, name):
        if self.busy or self.map_loading:
            return False, '档位或地图正在切换，请稍等'
        if self.prof('mapping') != 'run':
            return False, '保存地图需要「建图」正在运行'
        if not P.SAFE_NAME.match(name):
            return False, '地图名只能用字母、数字、下划线和减号'
        self.maps = P.list_maps(os.path.join(DATA, 'maps'))
        if any(m['name'] == name for m in self.maps):
            return False, f'已经有同名地图 {name}，换个名字（不会覆盖）'
        self.node.event('INFO', f'保存地图 {name}……')
        self.map_busy = True
        try:
            if await self.procs.save_map(name):
                self.maps = P.list_maps(os.path.join(DATA, 'maps'))
                self.node.event('INFO', f'地图已保存：data/maps/{name}')
                if MAPPING_VIEW in self.map_views:   # 建图时调好的显示方向带给新地图
                    await self.set_view(name, self.map_views[MAPPING_VIEW])
                return True, f'地图已保存：{name}'
            return False, '保存失败：' + (P.tail(os.path.join(DATA, 'logs', 'save_map.log'), 6) or '看 data/logs/save_map.log')
        finally:
            self.map_busy = False

    async def align_map(self, source, name, angle):
        if self.busy or self.map_loading:
            return False, '档位或地图正在切换，请稍等'
        if not all(isinstance(v, str) and P.SAFE_NAME.fullmatch(v) for v in (source, name)):
            return False, '源地图和目标地图名只能用字母、数字、下划线和减号（最多 48 字符）'
        if isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle) or abs(angle) > math.pi / 2:
            return False, '对齐角度必须在 -90° 到 90° 之间'
        self.maps = P.list_maps(os.path.join(DATA, 'maps'))
        if source not in [m['name'] for m in self.maps]:
            return False, '找不到已保存的原始地图'
        self.map_busy = True
        try:
            if not await self.procs.align_map(source, name, float(angle)):
                return False, '对齐导出失败，原图已保留：' + P.tail(os.path.join(DATA, 'logs/align_map.log'), 5)
            self.maps = P.list_maps(os.path.join(DATA, 'maps'))
            self.node.event('INFO', f'对齐地图已保存：{name}，原图 {source} 用于续建')
            return True, f'已保存 {name}（PGM / YAML / PNG），原图 {source} 保留用于续建'
        finally:
            self.map_busy = False

    async def load_map(self, name):
        n = self.node
        if name not in [m['name'] for m in self.maps]:
            return False, f'data/maps 里没有 {name}'
        if not n.loadmap_cli.service_is_ready():
            self.map_choice = name
            self.save_last_map(name)
            if self.preview_tick():
                return True, f'已切换到 {name}（导航启动时用这张图）'
            return True, f'下次启动定位 / 导航时用 {name}'
        if self.map_loading:
            return False, '正在切换地图'
        self.map_loading = True
        try:
            self.stop_route('切换地图')
            if n.nav_active():
                await self.aw(n.cancel_all(), 2.0)
            r = await self.aw(n.load_map(name), 10.0)
            if r.result != 0:
                return False, f'map_server 加载失败（错误码 {r.result}）'
            n.map_current = n.map_loaded = name
        finally:
            self.map_loading = False
        n.event('WARN', f'已切换到地图 {name}：请重新设置初始位姿')
        return True, f'已切换到 {name}，请重新设置初始位姿'

    # ---------- 导航没开时也能看地图 ----------
    def default_map(self):
        """启动定位 / 导航、或导航没开时显示用的地图：网页选的 > 正在用的 > 上次用的 > lab_20261021 > 最新的。"""
        names = [m['name'] for m in self.maps]
        for name in (self.map_choice, self.node.map_current, self.last_map, 'lab_20261021'):
            if name in names:
                return name
        return names[0] if names else None

    def save_last_map(self, name):
        if not name or name == self.last_map:
            return
        self.last_map = name
        try:
            os.makedirs(DATA, exist_ok=True)
            tmp = self.last_map_file + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump({'name': name}, f)
            os.replace(tmp, self.last_map_file)
        except OSError as e:
            self.node.get_logger().warn(f'记不住上次用的地图：{e!r}')

    def shown_map(self):
        """网页上现在显示的地图名：导航 / 定位在跑时是 map_server 的图，没开时是直接读的文件；建图中为 None。"""
        n = self.node
        return n.map_file if n.map_file is not None else n.map_current

    def preview_tick(self):
        """没有任何档位在跑时，把 default_map() 从文件读出来显示（换了选择或文件改了才重读）。返回是否在显示文件地图。"""
        n = self.node
        names = set(n.get_node_names())
        if any(self.prof(m) != 'stop' for m in P.MODES) or names & {'map_server', 'slam_toolbox'}:
            self.preview_key = None
            return False
        name = self.default_map()
        if not name:
            return False
        rec = next(m for m in self.maps if m['name'] == name)
        key = (name, rec['mtime'])
        if key == self.preview_key and n.map_file == name:
            return True
        self.preview_key = key
        try:
            m = mapfile.load(os.path.join(DATA, 'maps', f'{name}.yaml'))
        except Exception as e:
            n.event('WARN', f'读不了地图文件 {name}：{e}')
            return False
        n.show_file_map(name, m)
        return True

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
            try:
                await self.rollback_tick()
            except Exception as e:
                n.get_logger().warn(f'回溯录制启停失败：{e!r}')
            try:
                self.stuck_tick()
            except Exception as e:
                n.get_logger().warn(f'卡住检测失败：{e!r}')
            try:
                await self.bridge_tick()
            except Exception as e:
                n.get_logger().warn(f'遥控速度桥启停失败：{e!r}')
            names = set(n.get_node_names())
            for m, t0 in list(self.starting.items()):
                age = time.monotonic() - t0
                if all(x in names for x in MODE_NODES[m]):
                    self.starting.pop(m); self.transitions.pop(m, None)
                    n.event('INFO', f'{self.MODE_NAME[m]}已启动')
                    if AUTOLOC_ON_START and m in ('localization', 'navigation'):
                        self.background(self.auto_localize_on_start(m))
                elif self.procs_status is not None and not self.procs_status.get(m) and age > 4:
                    self.starting.pop(m); self.transitions.pop(m, None)
                    n.event('ERROR', f'{self.MODE_NAME[m]}启动失败，进程退出了。data/logs/{m}.log 最后几行：\n'
                            + P.tail(os.path.join(DATA, 'logs', f'{m}.log'), 8))
                elif age > START_TIMEOUT:
                    self.starting.pop(m); self.transitions.pop(m, None)
                    n.event('ERROR', f'{self.MODE_NAME[m]} {START_TIMEOUT:.0f} s 内没有起来，看 data/logs/{m}.log')
            if k % 3 == 1:
                self.maps = P.list_maps(os.path.join(DATA, 'maps'))
                try:
                    await self.bag_quota()   # 顺带刷新 self.bags
                except Exception as e:
                    self.bags = P.list_bags(os.path.join(DATA, 'bags'))
                    n.get_logger().warn(f'录包容量检查失败：{e!r}')
                n.fetch_map_current()
            if n.map_current and n.map_file is None:
                self.save_last_map(n.map_current)
            try:
                self.preview_tick()
            except Exception as e:
                n.get_logger().warn(f'显示地图文件失败：{e!r}')
            await asyncio.sleep(2.0)

    # ---------- 键盘遥控 ----------
    def on_teleop(self, ws, d):
        """网页按住按键时每 0.1 s 发一次；后端 0.3 s 收不到就发零速并停止发布。"""
        n, now = self.node, time.monotonic()
        if self.route_running() or self.checking_route:
            refuse = '多点任务或路线检查进行中，请先结束'
        else:
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
        remote = client_ip(req)
        self.clients[ws] = {'remote': remote, 'last_ping': time.monotonic(), 'layers': set()}
        self.node.get_logger().info(f'client connected: {remote} ({len(self.clients)})')
        try:
            await ws.send_str(self.routes_message())
            await ws.send_str(self.views_message())
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
                elif d.get('t') == 'cam':
                    r = await self.camera.request(ws, remote, bool(d.get('on')), bool(d.get('take')))
                    await ws.send_str(json.dumps({'t': 'cam', **r}))
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
            await self.camera.release(ws)
            self.clients.pop(ws, None)
            self.node.get_logger().info(f'client gone: {remote} ({len(self.clients)})')
            if ws is self.owner:
                self.owner = None
                await self.lose_owner(f'控制方网页断开（{remote}）')
        return ws

    async def lose_owner(self, reason):
        n = self.node
        if self.route_running() or n.is_armed() or n.nav_active():
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
                if self.route and self.route['status'] == 'active':
                    reason = self.route_blocker(self.owner, self.route['record'])
                    if reason:
                        await self.disarm_and_cancel(reason)
                elif k % 2 == 0 and n.nav_active() and self.ctrl_op != 'down':
                    # 单个导航目标 / 巡逻中机器狗掉进 Passive（遥控器 LT+B、摔倒保护）：取消，别让 Nav2 对着趴着的狗一直发速度
                    s, _, _ = self.ctrl_get()
                    if s is not None and s != 'Velocity':
                        await self.disarm_and_cancel(f'机器狗不在行走状态（运控状态 {s}）')
                if k % 5 == 0:
                    self.idle_tick()
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
                    await self.send_all(json.dumps({'t': 'task', 'task': self.task()}))
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
                    # mapCurrent = 网页上显示的地图（导航没开时是文件地图）；mapLive = 它来自 map_server，可以定位 / 规划
                    st.update(mapCurrent=self.shown_map(), mapLive=n.map_file is None and n.map_current is not None)
                    st.update(estop=self.estop, busy=self.busy, mapBusy=self.map_busy, maps=self.maps, bags=self.bags, uploads=self.uploads, mapChoice=self.map_choice,
                              rollback={'on': self.rollback_on, 'run': bool((self.procs_status or {}).get('rollback')), 'saving': self.rollback_saving, 'seconds': ROLLBACK_SECONDS},
                              procsKnown=self.procs_status is not None, teleop=self.teleop is not None, autoLoc=self.autoloc, ctrl=self.ctrl_view(),
                              bagLive=sorted(n for n in self.recording_names() if n) if self.prof('bag') != 'stop' else [],
                              bagSegment=P.BAG_SEGMENT if (self.procs_status or {}).get('bag_seg') else None)
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
                          asyncio.create_task(self.procs_poll()), asyncio.create_task(self.camera.monitor())]

        async def stop(a):
            if self.route_running():
                await self.disarm_and_cancel('网页服务关闭')
                await asyncio.gather(self.route_job, return_exceptions=True)
            await self.camera.release()
            for t in a['tasks']:
                t.cancel()

        app.on_startup.append(start)
        app.on_cleanup.append(stop)
        return app


# 经常驻服务器（192.168.1.166，nginx）转发的连接，req.remote 都是服务器的地址；
# 只有来自这些代理的请求才信任 X-Forwarded-For，别人发这个头伪造不了 IP
TRUSTED_PROXIES = set(filter(None, os.environ.get('GO2W_TRUSTED_PROXIES', '192.168.1.166').split(',')))


def client_ip(req):
    if req.remote in TRUSTED_PROXIES:
        fwd = req.headers.get('X-Forwarded-For', '').split(',')[-1].strip()
        if fwd:
            return f'{fwd} 经 {req.remote}'
    return req.remote


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
