#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import argparse, os, sys
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from imutypelib.config import Config
from imutypelib.stats import Stats

parser = argparse.ArgumentParser(description='Plot statistics')
parser.add_argument('config', metavar='CONFIG', help='config file')
parser.add_argument('-f', '--format', choices=('show', 'pdf', 'png', 'svg'),
        help='choose export format, or show plot window', default='show')
parser.add_argument('-s', '--start', type=int, help='start at epoch', default=1)
parser.add_argument('-e', '--end', type=int, help='end at epoch', default=0)
parser.add_argument('-O', '--output', help='output to this file')
parser.add_argument('-d', '--detailed', action='store_true', help='show all keys')
parser.add_argument('-S', '--smooth', type=int, help='smoothing level', default=0)
parser.add_argument('-t', '--title', type=str, help='set plot title')
args = parser.parse_args()

config = Config(args.config)
stats = Stats(config, detailed=args.detailed)
stats.load(config.epoch_stats_file)
print('loaded')

filename = None

if args.format != 'show':
    if args.output:
        filename = args.output
    else:
        j = config.epoch_stats_file
        name, _ = os.path.splitext(os.path.basename(j))
        filename = os.path.join(
            os.path.dirname(j),
            name + ('-detailed' if args.detailed else '') + '.' + args.format,
        )

stats.draw(filename=filename, start=args.start, end=args.end, smooth=args.smooth, title=args.title)
