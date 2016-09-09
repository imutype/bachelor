#!/usr/bin/env python2

# import os, sys
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import rospy, tf
import numpy as np
import tf.transformations as t
import imutype.msg as msg
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Header
# from geometry_msgs.msg import Quaternion, Vector3

rospy.init_node('export_visualization_data')

output = rospy.Publisher('/visualization_data', msg.VisualizationData)
listener = tf.TransformListener()

average_yaws = np.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
average_pitches = np.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

timer = rospy.Rate(100)

time = rospy.Time.now()

yaws = np.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
pitches = np.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
while not rospy.is_shutdown():
    now = rospy.Time.now()
    for i, frame in enumerate(['right_base', 'right_thumb', 'right_index', 'right_middle', 'right_ring', 'right_pinky']):
        try:
            (trans, rot) = listener.lookupTransform('/map' if frame == 'right_base' else '/right_base', frame, rospy.Time(0))
            _, pitches[i], yaws[i] = t.euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            continue

    dt = (now - time).to_sec()
    time = now

    f = dt * 0.1
    average_yaws = (1 - f) * average_yaws + f * np.array(yaws)
    average_pitches = (1 - f) * average_pitches + f * np.array(pitches)

    relative_yaws = yaws - average_yaws
    relative_pitches = pitches - average_pitches

    output.publish(now, yaws, pitches, average_yaws, average_pitches, relative_yaws, relative_pitches)
    timer.sleep()
