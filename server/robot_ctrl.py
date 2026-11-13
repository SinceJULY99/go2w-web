"""运控（go2w_deploy 的 go2w_ctrl）的状态和站起 / 趴下请求，域 0（Go2W 内网）。

go2w_ctrl 的接口（同事的仓库，部署在扩展坞 ~/go2w_deploy，见那边的 include/ctrl_state.h、sport_move_bridge.h）：
  - 状态 topic rt/go2w_ctrl/state（ROS2 里 /go2w_ctrl/state，std_msgs/String），10 Hz，变化时立刻补发：
      {"state":"Passive|FixStand|Velocity","ready":true,"seq":3}
      ready：Passive 进入即 true；FixStand 插值（2 s）走完才 true；Velocity 过了 blend_t 才 true
      seq：每切一次状态 +1（遥控器 LT+B、lowstate 超时、姿态过大掉进 Passive 也算）
  - 请求走原厂 sport 接口 rt/api/sport/request：
      1004 StandUp       Passive  → FixStand
      1002 BalanceStand  FixStand → Velocity（相当于遥控器 Start；FixStand 没站稳时也会接受，所以必须先等 ready）
      1005 StandDown     FixStand / Velocity → Passive（kp 0 + 阻尼，直接塌下去，不是慢慢坐下）
    回复 code 0 只表示收到；当前状态不接受的请求 0.2 s 后作废。切没切成以状态 topic 为准。

网页后端本身在域 43，这里像 go2w_cmd_bridge 一样另开一个域 0 的 context。
本地假 ROS（GO2W_FAKE_PROCS=1）没有多域，直接挂在主节点上，由假模拟器模拟运控。
"""
import json
import threading
import time

API_BALANCESTAND = 1002
API_STANDUP = 1004
API_STANDDOWN = 1005
STATES = ('Passive', 'FixStand', 'Velocity')


class CtrlLink:
    def __init__(self, main_node, event, fake=False):
        """event(level, msg)：写进网页日志。fake：本地假 ROS，挂在 main_node 上。"""
        from std_msgs.msg import String
        from unitree_api.msg import Request
        self.Request, self.event = Request, event
        self.snap = (None, False, -1, 0.0)    # (state, ready, seq, 收到时间 monotonic)；整体替换，跨线程读不用锁
        self.steady_t = None                  # 进入当前 (state, ready=true) 的时间；没 ready 时 None
        self.n = 0
        if fake:
            self.ctx, self.ros = None, main_node
        else:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.signals import SignalHandlerOptions
            self.ctx = Context()
            rclpy.init(context=self.ctx, domain_id=0, signal_handler_options=SignalHandlerOptions.NO)
            # 不接受命令行的 __node 重命名，否则会和域 43 的主节点同名
            self.ros = Node('go2w_web_robot', context=self.ctx, use_global_arguments=False)
            ex = SingleThreadedExecutor(context=self.ctx)
            ex.add_node(self.ros)
            threading.Thread(target=ex.spin, daemon=True).start()
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        # BEST_EFFORT 订阅能和可靠 / 不可靠的发布者都匹配上
        self.ros.create_subscription(String, '/go2w_ctrl/state', self.on_state,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pub = self.ros.create_publisher(Request, '/api/sport/request', 10)

    def on_state(self, m):
        try:
            d = json.loads(m.data)
            state, ready, seq = d['state'], bool(d['ready']), int(d['seq'])
        except (ValueError, KeyError, TypeError):
            return
        old = self.snap
        now = time.monotonic()
        if not ready or state != old[0] or seq != old[2] or not old[1]:
            self.steady_t = now if ready else None
        self.snap = (state, ready, seq, now)
        if old[0] is not None and (state != old[0] or seq != old[2]):
            self.event('INFO' if state != 'Passive' else 'WARN', f'运控状态：{old[0]} → {state}（seq {seq}）')

    def get(self, stale):
        """(state, ready, seq)；超过 stale 秒没收到状态时 state 为 None。"""
        state, ready, seq, t = self.snap
        if state is None or time.monotonic() - t > stale:
            return None, False, seq
        return state, ready, seq

    def last_rx(self):
        """最后一次收到运控状态的时间（monotonic）；从没收到过时 None。"""
        t = self.snap[3]
        return t if t > 0 else None

    def steady_since(self):
        """当前状态 ready 起的时间（monotonic）；没 ready 时 None。"""
        return self.steady_t

    def send(self, api_id):
        self.n += 1
        r = self.Request()
        r.header.identity.id = int(time.time() * 1e6) * 100 + self.n % 100
        r.header.identity.api_id = api_id
        r.header.policy.noreply = True    # 回复只表示「收到」，没用；以状态 topic 为准
        r.parameter = ''
        self.pub.publish(r)
