class LoadMap:
    class Request:
        def __init__(self):
            self.map_url = ''

    class Response:
        RESULT_SUCCESS, RESULT_MAP_DOES_NOT_EXIST = 0, 1

        def __init__(self, result=0):
            self.result = result
