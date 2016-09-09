import itertools

from .config import SamplingMode

IMU_NAMES = list(map(" ".join, itertools.product(('Right', 'Left'), ('base', 'thumb', 'index', 'middle', 'ring', 'pinky'))))

KEY_CODES = {
    0: 'NO KEY',
    20: 'T',
    21: 'Y',
    22: 'U',
    23: 'I',
    34: 'G',
    35: 'H',
    36: 'J',
    37: 'K',
    48: 'B',
    49: 'N',
    50: 'M',
    57: 'SPACE',
    105: 'LEFT',
    106: 'RIGHT',
}

AXIS_NAMES = {
    SamplingMode.full: ['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'qw', 'qx', 'qy', 'qz'],
    SamplingMode.quat: ['w', 'x', 'y', 'z'],
    SamplingMode.angles: ['pitch', 'yaw'],
    SamplingMode.quat_accel: ['w', 'x', 'y', 'z', 'ax', 'ay', 'az'],
}
