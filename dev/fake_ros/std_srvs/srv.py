class SetBool:
    class Request:
        def __init__(self):
            self.data = False

    class Response:
        def __init__(self, success=False, message=''):
            self.success, self.message = success, message
