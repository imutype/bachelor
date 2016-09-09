#!/usr/bin/env python2
"""
This node converts our ImuEvent to sensor_msgs/Imu messages and tf transforms,
so rviz understands them.
"""

import os, sys, rospy, tf, itertools
import imutype.msg as imutype_msgs
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
import sensor_msgs.msg as sensor_msgs

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from imutypelib.config import Config, SamplingMode
from imutypelib.components import merge_events, preprocess
from imutypelib.utils import coroutine

rospy.init_node('convert_imu_events')

COVARIANCE_IDENTITY = (1, 0, 0, 0, 1, 0, 0, 0, 1) # diagonal 3x3 matrix

@coroutine
def publisher():
    broadcaster = tf.TransformBroadcaster()
    outputs = {}
    sequence_counter = itertools.count()
    anglesoutput = rospy.Publisher('/angles', imutype_msgs.Angles)

    while True:
        time, nodes = (yield)

        angles = [0, 0] * len(nodes)

        for i, node in enumerate(nodes):
            if not node.name in outputs:
                outputs[node.name] = rospy.Publisher('/imu_sensors/{}'.format(node.name), sensor_msgs.Imu)

            # quat = node.quat_relative_to_average if node.name[-5:] == '_base' else node.quat_relative
            angles[i*2:i*2+2] = node.get_sample(SamplingMode.angles, i==0)

            quat = node.quat_relative_to_average if i in (0, 6) else node.quat_relative
            quat = (quat[1], quat[2], quat[3], quat[0])

            outputs[node.name].publish(
                # header
                Header(
                    next(sequence_counter),
                    time,
                    node.parent.name if node.parent else '',
                ),

                # orientation
                Quaternion(*quat),
                COVARIANCE_IDENTITY,

                # angular_velocity
                Vector3(*node.gyro),
                COVARIANCE_IDENTITY,

                # linear_acceleration
                Vector3(*node.accel),
                COVARIANCE_IDENTITY,
            )

            broadcaster.sendTransform(
                node.offset,
                quat,
                time, # rospy.Time.now(),
                node.name,
                node.parent.name if node.parent else '',
            )

        anglesoutput.publish(angles)

if __name__ == '__main__':
    config = Config(rospy.get_param('/config_file'))
    pipeline = merge_events(config, preprocess(config, nodes_target=publisher()))

    def onImuEvent(event):
        try:
            pipeline.send(('/imu_events', event))
        except StopIteration as e:
            sys.exit(1)

    rospy.Subscriber('/imu_events', imutype_msgs.ImuEvent, onImuEvent, queue_size=1, tcp_nodelay=True)
    rospy.spin()
