#!/usr/bin/env python2
import rosbag, sys, collections
import numpy as np
import tf.transformations as t


bag = rosbag.Bag(sys.argv[1])

internal_eulers = {}
madgwick_eulers = {}
gyros = {}

def euler(q):
    return t.euler_from_quaternion((q.x, q.y, q.z, q.w))

for topic, msg, time in bag.read_messages(topics=['/imus/0/imu', '/imu/data']):
    time = msg.header.stamp.to_sec()
    if topic == '/imus/0/imu':
        internal_eulers[time] = euler(msg.orientation)
        v = msg.angular_velocity
        gyros[time] = (v.x, v.y, v.z)
    if topic == '/imu/data':
        madgwick_eulers[time] = euler(msg.orientation)

times = sorted(list(set(madgwick_eulers.keys()) & set(internal_eulers.keys())))

values = [[t] + list(gyros[t]) + list(internal_eulers[t]) + list(madgwick_eulers[t]) for t in times]
arr = np.array(values)
np.save(sys.argv[2], arr)
