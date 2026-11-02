from rclpy._fake import Future


class ActionClient:
    """只支持 go2w_web 用到的 navigate_to_pose：发目标、反馈、结果。"""

    def __init__(self, node, _type, name):
        self.node, self.name = node, name

    def server_is_ready(self):
        sim = getattr(self.node, 'fake_sim', None)
        return sim is not None and sim.nav_ready()

    def send_goal_async(self, goal, feedback_callback=None):
        fut = Future()
        sim = self.node.fake_sim
        sim.later(lambda: fut.set_result(sim.nav_send_goal(goal, feedback_callback)))
        return fut
