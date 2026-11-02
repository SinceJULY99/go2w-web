"""假 rclpy：只实现 web/server/go2w_web.py 用到的接口，数据来自 go2w_fake_sim。

只用于在 Mac 上本地调试（web/dev/run.sh），不要放进容器的 PYTHONPATH。
"""


def init():
    pass


def try_shutdown():
    pass


def spin(node):
    import go2w_fake_sim
    go2w_fake_sim.run(node)
