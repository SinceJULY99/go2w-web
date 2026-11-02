class Log:
    DEBUG, INFO, WARN, ERROR, FATAL = 10, 20, 30, 40, 50

    def __init__(self, level=20, name='', msg=''):
        self.level, self.name, self.msg = level, name, msg


class ParameterType:
    PARAMETER_NOT_SET = 0
    PARAMETER_BOOL = 1
    PARAMETER_INTEGER = 2
    PARAMETER_DOUBLE = 3


class ParameterValue:
    def __init__(self, type=0, double_value=0.0, integer_value=0, string_value=''):
        self.type, self.double_value, self.integer_value, self.string_value = type, double_value, integer_value, string_value


class Parameter:
    def __init__(self):
        self.name = ''
        self.value = ParameterValue()


class SetParametersResult:
    def __init__(self, successful=True, reason=''):
        self.successful, self.reason = successful, reason
