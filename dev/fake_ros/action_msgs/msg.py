from types import SimpleNamespace as N


class GoalStatus:
    STATUS_UNKNOWN = 0
    STATUS_ACCEPTED = 1
    STATUS_EXECUTING = 2
    STATUS_CANCELING = 3
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6


class GoalStatusArray:
    def __init__(self, items=()):
        # items: [(uuid bytes, status)]
        self.status_list = [N(goal_info=N(goal_id=N(uuid=list(u))), status=s) for u, s in items]
