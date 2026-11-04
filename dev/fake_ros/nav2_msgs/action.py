from geometry_msgs.msg import PoseStamped


class NavigateToPose:
    class Goal:
        def __init__(self, pose=None):
            self.pose = pose or PoseStamped()
            self.behavior_tree = ''

    class Impl:
        class FeedbackMessage:
            """/navigate_to_pose/_action/feedback 上的消息：goal_id + feedback。"""


class ComputePathToPose:
    class Goal:
        def __init__(self):
            self.goal = PoseStamped()
            self.start = PoseStamped()
            self.use_start = False
            self.planner_id = ''
