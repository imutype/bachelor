#!/usr/bin/env python2

import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import evdev
from evdev import uinput, ecodes as e
import rospy
import imutype.msg as msg
import numpy as np
import sys
# from xdo import Xdo, CURRENTWINDOW, charcodemap_t
# xdo = Xdo()
rospy.init_node('export_training_data', disable_signals=True)


with uinput.UInput() as ui:
    def onPredictedKeyEvent(event):
        predicted_key_event = event
        # charcodemap = (charcodemap_t * 1)(charcodemap_t(code=chr(predicted_key_event.code)))
        # xdo.send_keysequence_window_list_do(CURRENTWINDOW, charcodemap, pressed=1 if predicted_key_event.pressed else 0)
        ui.write(e.EV_KEY, predicted_key_event.code, predicted_key_event.pressed)
        # ui.write(e.EV_KEY, e.KEY_A, 1)
        ui.syn()

    rospy.Subscriber('/predicted_key_events', msg.KeyEvent, onPredictedKeyEvent, queue_size=1, tcp_nodelay=True)
    rospy.spin()

