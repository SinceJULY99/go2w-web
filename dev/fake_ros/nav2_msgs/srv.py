class LoadMap:
    class Request:
        def __init__(self):
            self.map_url = ''

    class Response:
        RESULT_SUCCESS, RESULT_MAP_DOES_NOT_EXIST = 0, 1

        def __init__(self, result=0):
            self.result = result


class ClearEntireCostmap:
    class Request:
        pass

    class Response:
        def __init__(self):
            pass


class ClearCostmapExceptRegion:
    class Request:
        def __init__(self, reset_distance=0.0):
            self.reset_distance = reset_distance

    class Response:
        def __init__(self):
            pass
