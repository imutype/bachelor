import tf.transformations as t
import numpy as np
from imutypelib.config import SamplingMode
from imutypelib.utils import coroutine
import math

# mat4x4 aFromB = /* construct this by hand */;
# mat4x4 bFromA = aFromB.inverse();
# mat4x4 transformInA = ...;
# mat4x4 transformInB = bFromA * transformInA * aFromB;

# http://answers.ros.org/question/9659/quaternion-from-imu-interpreted-incorrectly-by-ros/

class Node(object):
    def __init__(self, name, parent = None, offset = (0, 0, 0), relative_average_rate=0):
        self.lastUpdatedAt = None
        self.name = name
        self.parent = parent
        self.offset = offset
        self.quat_relative = (1, 0, 0, 0)
        self.quat_absolute = (1, 0, 0, 0)
        self.quat_average = None
        self.quat_relative_to_average = (1, 0, 0, 0)
        self.accel = (0, 0, 0)
        self.gyro = (0, 0, 0)
        self.relative_average_rate = relative_average_rate

    def update(self, now, accel, gyro, quat):
        self.accel = accel
        self.gyro = gyro

        w, x, y, z = quat
        # w, x, y, z = -w, y, x, z
        w, x, y, z = w, -y, x, z
        quat = w, x, y, z

        # quat = t.quaternion_multiply(t.quaternion_about_axis(math.pi, [0, 0, 1]), quat)

        self.quat_absolute = quat

        # create relative quat, based on our absolute quaternion and our parent's absolute quaternion
        self.quat_relative = (
            t.quaternion_multiply(self.quat_absolute, t.quaternion_inverse(self.parent.quat_absolute))
            if self.parent else
            self.quat_absolute
        )
        self.quat_relative = t.quaternion_multiply(self.quat_relative, t.quaternion_about_axis(math.pi, [1, 0, 0]))

        # euler = t.euler_from_quaternion(quat_relative, axes='syzx')
        # quat_relative = t.quaternion_from_euler(euler[0], euler[1], -euler[2], axes='sxyz')
        # self.quat_relative = t.quaternion_multiply(quat_relative, t.quaternion_inverse(self.parent.quat_relative))

        if self.lastUpdatedAt and self.relative_average_rate:
            dt = (now - self.lastUpdatedAt).to_sec()
            f = dt * self.relative_average_rate
            self.quat_average = self.quat_relative \
                if self.quat_average is None \
                else t.quaternion_slerp(self.quat_average, self.quat_relative, f)

            self.quat_relative_to_average = t.quaternion_multiply(self.quat_relative, t.quaternion_inverse(self.quat_average))
        else:
            self.quat_average = self.quat_relative
            self.quat_relative_to_average = self.quat_relative

        self.lastUpdatedAt = now

    def get_sample(self, mode, use_relative=False):
        if mode == SamplingMode.full:
            return np.concatenate([self.accel, self.gyro, self.quat_relative])
        elif mode == SamplingMode.quat:
            quat = self.quat_relative_to_average if use_relative else self.quat_relative
            return quat
        elif mode == SamplingMode.angles:
            quat = self.quat_relative_to_average if use_relative else self.quat_relative
            yaw, pitch, _ = t.euler_from_quaternion(quat)
            return np.array([pitch, yaw])
        elif mode == SamplingMode.quat_accel:
            quat = self.quat_relative_to_average if use_relative else self.quat_relative
            return np.concatenate([quat, self.accel])

@coroutine
def preprocess(config, target=None, nodes_target=None):
    map_node = Node('map')
    right_base = Node('right_base', map_node, (0, 0, 0), config.base_imu_relative_average_rate)
    left_base = Node('left_base', map_node, (0, 0.1, 0), config.base_imu_relative_average_rate)

    fr = config.finger_imu_relative_average_rate

    nodes = [
        right_base,
        Node('right_thumb', right_base, (0.035, 0.065, 0), fr),
        Node('right_index', right_base, (0.10, 0.015, 0), fr),
        Node('right_middle', right_base, (0.105, -0.010, 0), fr),
        Node('right_ring', right_base, (0.10, -0.0450, 0), fr),
        Node('right_pinky', right_base, (0.07, -0.0750, 0), fr),

        left_base,
        Node('left_thumb', left_base, (0.035, 0.065, 0), fr),
        Node('left_index', left_base, (0.10, 0.015, 0), fr),
        Node('left_middle', left_base, (0.105, -0.010, 0), fr),
        Node('left_ring', left_base, (0.10, -0.0450, 0), fr),
        Node('left_pinky', left_base, (0.07, -0.0750, 0), fr),
    ]

    while True:
        time, data = (yield)
        for i, imu_id in enumerate(config.imu_ids):
            values = data[i * 10:i * 10 + 10]
            nodes[imu_id].update(time, values[0:3], values[3:6], values[6:10])

        if nodes_target:
            nodes_target.send((time, nodes))

        if target:
            unprocessed_key_states = data[len(config.imu_ids) * 10:]

            processed_imu_states = np.concatenate([
                nodes[imu_id].get_sample(
                    mode=config.sampling_mode,
                    use_relative=imu_id in (0,6) or fr,
                )
                for imu_id
                in config.imu_ids
            ])

            target.send((time, np.concatenate((processed_imu_states, unprocessed_key_states))))
