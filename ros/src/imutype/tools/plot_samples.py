#!/usr/bin/env python2
from __future__ import print_function
import os, sys, random
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import rosbag, sys, itertools
import matplotlib.pyplot as plt
import tf.transformations as t
import sklearn.preprocessing
from os.path import splitext
import numpy as np

from imutypelib import KEY_CODES, IMU_NAMES, Config, get_samples, AXIS_NAMES

MAX_SAMPLES_PER_PLOT = 100

####################################################################333
# configure plots matrix

BASE, THUMB, INDEX = 0, 1, 2
PITCH, YAW = 0, 1
H, N = 35, 49

DRAW_MATRIX = False
DRAW_AVERAGE = True

#
####################################################################333

def plot(config, ax, samples, imu_id, key_code, value_index):
    samples = np.array([s[1] for s in samples if s[2] == key_code])
    print('key_code: {}, imu: {}, value index: {}, samples: {}'.format(key_code, imu_id, value_index, len(samples)))

    if imu_id == -1:
        title = KEY_CODES[key_code] if value_index == 0 else None
    else:
        title = '{} | {} | {}'.format(
            KEY_CODES[key_code],
            IMU_NAMES[imu_id],
            AXIS_NAMES[config.sampling_mode][value_index],
        )

    if title:
        ax.set_title(title)

    color = (226.0/255,0,26.0/255, 0.2)

    ax.axvline(0, color=(0, 0, 0, 0.2))

    before = int(round(config.sequence_length * config.sequence_ratio))
    after = config.sequence_length - before
    x = [((i + 1) * 1.0 / config.sampling_rate) for i in range(-before, after)]

    lines = []
    if imu_id == -1:
        for i, imu_id in enumerate(config.imu_ids):
            index = i * config.sampling_mode + value_index
            y = np.average(samples[:,:,index], axis=0)

            line, = ax.plot(x, y, lw=3)
            lines.append(line)
    else:
        for sample in samples[:MAX_SAMPLES_PER_PLOT]:
            index = config.imu_ids.index(imu_id) * config.sampling_mode + value_index
            y = [step[index] for step in sample]

            line, = ax.plot(x, y, color=color)
            lines.append(line)

    return lines

if __name__ == '__main__':
    config = Config(sys.argv[1])

    # TODO: remove
    config.key_codes = [20, 34, 21, 35, 22, 36]

    with np.load(config.samples_file) as npzfile:
        samples = npzfile['samples']

    if DRAW_MATRIX:
        # _, axes = plt.subplots(1, 1 + len(config.key_codes))
        # for i, k in enumerate(config.key_codes[0]):
        #     key_samples = [s[1] for s in samples if s[2] == k]
        #     sample = np.average(key_samples, axis=0)
        #
        #     ax = axes[i]
        #     ax.set_title('{}'.format(KEY_CODES[k]))
        #     sample = sklearn.preprocessing.minmax_scale(sample, axis=0)
        #     ax.matshow(sample.T, cmap=plt.cm.gray)

        sample = samples[100][1]
        sample = sklearn.preprocessing.minmax_scale(sample, axis=0)
        plt.matshow(sample.T, cmap=plt.cm.gray)
        plt.savefig('matrix.png', dpi=300)
        plt.show()
    else:
        if DRAW_AVERAGE:
            PLOTS = [
                [(-1, k, i) for k in config.key_codes] for i in range(config.sampling_mode)
            ]

        else:
            PLOTS = [
                [(INDEX, k, i) for k in config.key_codes] for i in range(config.sampling_mode)
            ]
        # PLOTS = [
        #     [(INDEX, N, PITCH)],
        # ]


        rows = len(PLOTS)
        cols = max(map(len, PLOTS))

        plt.rc('axes', titlesize=20, labelsize=20)
        plt.rc('xtick', labelsize=16)
        plt.rc('ytick', labelsize=16)
        plt.rc('legend', fontsize=16)

        _, axes = plt.subplots(rows, cols, sharex=True, sharey='row')

        for i, row in enumerate(PLOTS):
            for j, args in enumerate(row):
                ax = axes
                if rows > 1: ax = ax[i]
                if cols > 1: ax = ax[j]

                if j == 0:
                    ax.set_ylabel(AXIS_NAMES[config.sampling_mode][args[2]])
                if i == rows - 1:
                    ax.set_xlabel('time (s)')

                lines = plot(config, ax, samples, *args)

        figure = plt.gcf()
        figure.legend(lines, [IMU_NAMES[i] for i in config.imu_ids], loc= 'upper center', ncol=len(config.imu_ids), labelspacing=0.0)

        figure.set_size_inches(3 * cols, 3 * rows)
        figure.tight_layout(rect=[0,0,1,0.95])
        plt.savefig(config.samples_plot_file, dpi=72)
        plt.show()

