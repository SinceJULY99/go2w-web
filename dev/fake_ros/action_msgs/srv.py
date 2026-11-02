from types import SimpleNamespace as N


class CancelGoal:
    class Request:
        def __init__(self):
            # 零 uuid + 零时间戳 = 取消全部
            self.goal_info = N(goal_id=N(uuid=[0] * 16), stamp=N(sec=0, nanosec=0))

    class Response:
        def __init__(self):
            self.return_code = 0
            self.goals_canceling = []
