class GetParameters:
    class Request:
        def __init__(self):
            self.names = []

    class Response:
        def __init__(self, values=()):
            self.values = list(values)


class SetParameters:
    class Request:
        def __init__(self):
            self.parameters = []

    class Response:
        def __init__(self, results=()):
            self.results = list(results)
