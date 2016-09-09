#!/usr/bin/env python2
from __future__ import print_function
import os, sys, random
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import rosbag, sys, itertools
import matplotlib.pyplot as plt
import tf.transformations as t
import numpy as np

from imutypelib import Config, get_samples

if __name__ == '__main__':
    config = Config(sys.argv[1])
    samples = get_samples(config)
    np.savez(config.samples_file, samples=samples)
