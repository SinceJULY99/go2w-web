"""假 unitree_api：只有网页后端发站起 / 趴下请求用到的字段。"""
from types import SimpleNamespace as N


class Request:
    def __init__(self):
        self.header = N(identity=N(id=0, api_id=0), lease=N(id=0), policy=N(priority=0, noreply=False))
        self.parameter = ''
        self.binary = []
