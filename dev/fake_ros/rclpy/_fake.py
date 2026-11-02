"""假 rclpy 的内部工具：Future、发布者、服务客户端。服务和 action 的处理逻辑在 go2w_fake_sim 里。"""
import threading


class Future:
    def __init__(self):
        self._done, self._result, self._cbs = False, None, []
        self._lock = threading.Lock()

    def set_result(self, r):
        with self._lock:
            if self._done:
                return
            self._done, self._result = True, r
            cbs = list(self._cbs)
        for cb in cbs:
            cb(self)

    def done(self):
        return self._done

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        with self._lock:
            if not self._done:
                self._cbs.append(cb)
                return
        cb(self)


class Publisher:
    def __init__(self, node, topic):
        self.node, self.topic = node, topic

    def publish(self, msg):
        handler = getattr(self.node, 'fake_sim', None)
        if handler is not None:
            handler.on_publish(self.topic, msg)

    def get_subscription_count(self):
        sim = getattr(self.node, 'fake_sim', None)
        return sim.subscribers(self.topic) if sim is not None else 0


class Client:
    def __init__(self, node, name):
        self.node, self.name = node, name

    def service_is_ready(self):
        sim = getattr(self.node, 'fake_sim', None)
        return sim is not None and sim.has_service(self.name)

    def call_async(self, req):
        fut = Future()
        sim = self.node.fake_sim
        # 真服务的回复是异步的：放到模拟器线程里处理，回调不在调用方线程触发
        sim.later(lambda: fut.set_result(sim.call_service(self.name, req)))
        return fut
