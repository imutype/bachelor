#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import argparse, os, sys
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from imutypelib.config import Config
from imutypelib.stats import Stats

config = Config(sys.argv[1])
stats = Stats(config)
stats.load(config.epoch_stats_file.replace('.npz', '.json'))
print('loaded')
stats.save(config.epoch_stats_file)
print('saved')
