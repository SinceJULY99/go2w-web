from rclpy._fake import Future


class ActionClient:
    """模拟网页用到的导航和路径检查 action。"""

    def __init__(self, node, _type, name):
        self.node, self.name = node, name

    def server_is_ready(self):
        sim = getattr(self.node, 'fake_sim', None)
        return sim is not None and sim.nav_ready()

    def send_goal_async(self, goal, feedback_callback=None):
        fut = Future()
        sim = self.node.fake_sim
        if self.name == 'compute_path_to_pose':
            sim.later(lambda: fut.set_result(sim.compute_path(goal)))
        else:
            sim.later(lambda: fut.set_result(sim.nav_send_goal(goal, feedback_callback)))
        return fut
