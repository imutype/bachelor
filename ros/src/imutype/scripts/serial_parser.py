#!/usr/bin/env python2

import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import rospy
import imutype.msg as msg
import imutypelib.serialparser as parser

rospy.init_node('serial_parser')
keyEventTopic = rospy.Publisher('/key_events', msg.KeyEvent, queue_size=1, tcp_nodelay=True)
imuEventTopic = rospy.Publisher('/imu_events', msg.ImuEvent, queue_size=1, tcp_nodelay=True)

data_source = rospy.get_param("~data_source", None)
from_network = rospy.get_param("~from_network", False)

for event in parser.Parser(data_source, from_network):
    if rospy.is_shutdown():
        break

    if isinstance(event, parser.ImuEvent):
        imuEventTopic.publish(rospy.Time.now(), event.imuId, msg.SensorState(event.accel, event.gyro, event.quat))

    elif isinstance(event, parser.Message):
        print('[{}] {}'.format(event.code, event.message))

    elif isinstance(event, parser.KeyEvent):
        keyEventTopic.publish(rospy.Time.now(), event.keyCode, event.pressed)
