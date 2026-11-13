"""假 rclpy.executors：go2w_web.py 用 SingleThreadedExecutor 跑主节点，这里转给 go2w_fake_sim。"""


class ExternalShutdownException(Exception):
    pass


class SingleThreadedExecutor:
    def __init__(self, context=None):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)

    def spin(self):
        import go2w_fake_sim
        go2w_fake_sim.run(self.nodes[0])

    def shutdown(self):
        pass
