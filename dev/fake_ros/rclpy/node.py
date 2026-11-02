import sys
import time
from types import SimpleNamespace as N

from rclpy._fake import Client, Publisher

# topic 清单（和导航运行时一致）；有没有发布者由假模拟器按「哪些进程在跑」决定
TOPICS = [
    ('/livox/lidar', 'livox_ros_driver2/msg/CustomMsg'), ('/livox/imu', 'sensor_msgs/msg/Imu'),
    ('/cloud_registered_body', 'sensor_msgs/msg/PointCloud2'), ('/scan', 'sensor_msgs/msg/LaserScan'),
    ('/odom', 'nav_msgs/msg/Odometry'), ('/odom_healthy', 'std_msgs/msg/Bool'), ('/tf', 'tf2_msgs/msg/TFMessage'),
    ('/map', 'nav_msgs/msg/OccupancyGrid'), ('/amcl_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped'),
    ('/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped'),
    ('/plan', 'nav_msgs/msg/Path'), ('/cmd_vel', 'geometry_msgs/msg/Twist'),
    ('/global_costmap/costmap', 'nav_msgs/msg/OccupancyGrid'), ('/local_costmap/costmap', 'nav_msgs/msg/OccupancyGrid'),
    ('/particle_cloud', 'nav2_msgs/msg/ParticleCloud'),
    ('/behavior_tree_log', 'nav2_msgs/msg/BehaviorTreeLog'),
    ('/go2w_cmd_bridge/armed', 'std_msgs/msg/Bool'), ('/patrol_pose', 'geometry_msgs/msg/PoseStamped'),
    ('/rosout', 'rcl_interfaces/msg/Log'),
]


class _Logger:
    """和真 rclpy 一样：同一处调用（文件 + 行号）不能先后用不同级别，否则抛 ValueError。"""
    _sites = {}

    def __init__(self, name):
        self.name = name

    def _p(self, lv, msg):
        f = sys._getframe(2)
        site = (f.f_code.co_filename, f.f_lineno)
        if self._sites.setdefault(site, lv) != lv:
            raise ValueError('Logger severity cannot be changed between calls.')
        print(f'[{lv}] [{time.strftime("%H:%M:%S")}] [{self.name}]: {msg}', flush=True)

    def info(self, msg): self._p('INFO', msg)
    def warn(self, msg): self._p('WARN', msg)
    def error(self, msg): self._p('ERROR', msg)

    def warning(self, msg): self._p('WARN', msg)


class _Clock:
    def now(self):
        t = time.time()
        return N(to_msg=lambda: N(sec=int(t), nanosec=int((t % 1) * 1e9)))


class Node:
    def __init__(self, name):
        self._name = name
        self._subs = {}
        self._timers = []
        self._log = _Logger(name)

    def get_logger(self):
        return self._log

    def get_clock(self):
        return _Clock()

    def create_subscription(self, _type, topic, cb, _qos):
        h = N(topic=topic, cb=cb)
        self._subs.setdefault(topic, []).append(h)
        return h

    def destroy_subscription(self, h):
        lst = self._subs.get(h.topic, [])
        if h in lst:
            lst.remove(h)

    def create_publisher(self, _type, topic, _qos):
        return Publisher(self, topic)

    def create_client(self, _type, name):
        return Client(self, name)

    def create_timer(self, _period, cb):   # 假模拟器每 50 ms 调一次所有定时器
        self._timers.append(cb)

    def deliver(self, topic, msg):  # 假模拟器调用：相当于 DDS 把消息送到订阅回调
        for h in list(self._subs.get(topic, [])):
            h.cb(msg)

    def get_node_names(self):
        sim = getattr(self, 'fake_sim', None)
        return sim.node_names() if sim else []

    def get_topic_names_and_types(self):
        return [(n, [t]) for n, t in TOPICS]

    def count_publishers(self, topic):
        sim = getattr(self, 'fake_sim', None)
        return 1 if sim and sim.topic_alive(topic) else 0

    def destroy_node(self):
        pass
