"""假 rosidl_runtime_py：message_to_yaml 把假消息（SimpleNamespace / 普通对象）转成和真 ROS 差不多的 YAML。"""


def _dump(v, indent, trunc):
    pad = '  ' * indent
    if isinstance(v, (bytes, bytearray)):
        v = list(v)
    if isinstance(v, (list, tuple)):
        if trunc and len(v) > trunc:
            v = list(v[:trunc]) + ['...']
        if not v or not hasattr(v[0], '__dict__'):
            return ' [' + ', '.join(_scalar(x) for x in v) + ']\n'
        out = '\n'
        for x in v:   # 对象列表：每项第一行前面加「- 」，其余行的缩进正好对齐
            lines = _dump(x, indent + 1, trunc).strip('\n').split('\n')
            out += f'{pad}- {lines[0].strip()}\n' + ''.join(line + '\n' for line in lines[1:])
        return out
    if hasattr(v, '__dict__'):
        items = [(k, x) for k, x in vars(v).items() if not k.startswith('_')]
        return '\n' + ''.join(f'{pad}{k}:' + _dump(x, indent + 1, trunc) for k, x in items)
    return ' ' + _scalar(v) + '\n'


def _scalar(x):
    if isinstance(x, float):
        return repr(round(x, 6))
    if isinstance(x, str):
        return x if x and all(c.isalnum() or c in '_/.-' for c in x) else repr(x)
    return str(x)


def message_to_yaml(msg, truncate_length=None, no_arr=False, no_str=False):
    return _dump(msg, 0, truncate_length).lstrip('\n')
