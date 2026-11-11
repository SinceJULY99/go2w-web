class DurabilityPolicy:
    TRANSIENT_LOCAL = 'transient_local'


class ReliabilityPolicy:
    RELIABLE = 'reliable'
    BEST_EFFORT = 'best_effort'


def QoSProfile(**kw):
    return kw


qos_profile_sensor_data = {}
