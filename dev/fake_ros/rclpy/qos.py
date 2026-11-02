class DurabilityPolicy:
    TRANSIENT_LOCAL = 'transient_local'


class ReliabilityPolicy:
    RELIABLE = 'reliable'


def QoSProfile(**kw):
    return kw


qos_profile_sensor_data = {}
