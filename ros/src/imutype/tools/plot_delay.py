#!/usr/bin/env python2
from __future__ import print_function
import os, sys, random
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import rosbag
import numpy as np
import matplotlib.pyplot as plt

SPACE = 57

accx = []
keys = []

with rosbag.Bag(sys.argv[1]) as bag:
    for topic, msg, time in bag.read_messages(topics=['/imu_events', '/key_events']):
        if topic == '/imu_events':
            if msg.imuId == 0:
                accx.append((msg.time.to_sec(), msg.data.accel[2]))
        elif topic == '/key_events':
            keys.append(time.to_sec())


maxs = []
for t in keys[2:3]:
    region = sorted([(a[0]-t, a[1]) for a in accx if abs(a[0] - t) < 0.05])

    x, y = zip(*region)
    plt.plot(x, y)

    m = region[np.argmax(y)]
    if m[0] > -0.05:
        maxs.append(m)

plt.scatter(*zip(*maxs))
print(np.average(np.array(maxs)[:,0]))
plt.axvline(0, color=(0, 0, 0, 0.2))
plt.xlabel('time (s)')
plt.ylabel('z acc (m/s^2)')
plt.gcf().set_size_inches(8, 6)
plt.savefig('delay.png', dpi=72)
plt.show()
