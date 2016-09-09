#!/usr/bin/env python2
import matplotlib.pyplot as plt
import numpy as np
import math

arr = np.load('heading.npy').T

# cut off first third
print(arr.shape)
arr = arr[:,arr.shape[1]//3:]
print(arr.shape)

X = arr[0]
X = X - np.min(X)

for y in (6, 9):
    Y = arr[y]
    Y = np.where(Y < 0, Y + 2 * math.pi, Y)
    arr[y] = Y - Y[0]
plt.figure(1)

# AXES = ('yaw', 'pitch', 'roll')
AXES = ('euler x', 'euler y', 'euler z')

ax1 = plt.subplot(411)
plt.title('gyro')
lines = [plt.plot(X, y, label=label)[0] for label, y in zip(('x', 'y', 'z'), arr[1:4])]
plt.legend(handles=lines)

for i in range(3):
    plt.subplot(412+i, sharex=ax1)
    plt.title(AXES[i])
    lines = [plt.plot(X, y, label=label)[0] for label, y in zip(('internal', 'madgwick'), arr[4+i:10+i:3])]
    plt.legend(handles=lines)
    plt.axhline(0, color=(0, 0, 0, 0.2))

# plt.subplot(313, sharex=ax1)
# plt.title('madgwick')
# lines = [plt.plot(X, y, label=label)[0] for label, y in zip(AXES, arr[7:10])]
# plt.legend(handles=lines)

# plt.xlim([120, 350])

fig = plt.gcf()
fig.tight_layout(pad=0,h_pad=0,w_pad=0)

fig.set_size_inches(20, 16)
fig.tight_layout()
plt.savefig('heading.png', dpi=72)
plt.show()
