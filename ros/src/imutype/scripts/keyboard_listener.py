#!/usr/bin/env python2
"""
This node reads evdev events from the keyboard input device specified as the only parameter, and sends them as
KeyEvent messages to the /key_events topic.
"""

import rospy
import imutype.msg as msg
import evdev

# argv = rospy.myargv()
# assert len(argv) == 2, "Needs exactly 1 parameter: keyboard input device"

rospy.init_node('keyboard_listener')
keyEventTopic = rospy.Publisher('/key_events', msg.KeyEvent, queue_size=1, tcp_nodelay=True)

# dev = evdev.InputDevice(argv[1])
param = rospy.get_param("~input_device")
dev = evdev.InputDevice(param)
rospy.loginfo("Reading keyboard events from EVDEV {}".format(dev))

for event in dev.read_loop():
    if event.type == evdev.ecodes.EV_KEY:
        keyEvent = evdev.KeyEvent(event)

        if keyEvent.keystate != evdev.KeyEvent.key_hold:
            now = rospy.Time.now()
            code = keyEvent.scancode
            pressed = (keyEvent.keystate == evdev.KeyEvent.key_down)

            keyEventTopic.publish(now, code, pressed)
