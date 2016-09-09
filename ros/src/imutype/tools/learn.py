#!/usr/bin/env python2
from __future__ import print_function
import os, sys, random
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

random.seed(0) # let's have some deterministic shuffling

import numpy as np
import theano
import itertools
from imutypelib.config import Config
from imutypelib.network import Network
from imutypelib.stats import Stats
from imutypelib.pipeline import get_samples
from imutypelib.constants import KEY_CODES

def divide_test_training_data(data, ratio):
    s = int(ratio * len(data))
    return data[:s], data[s:]

def build_batch(config, samples, size=None):
    indices = [random.randint(0, len(samples)-1) for i in range(size)] if size else list(range(len(samples)))
    random.shuffle(indices)
    inputs, outputs = [], []
    for i in indices:
        inputs.append(samples[i][1])
        outputs.append([(1 if key_code == samples[i][2] else 0) for key_code in [0] + config.key_codes])

    return (
            np.array(inputs, dtype=theano.config.floatX),
            np.array(outputs, dtype=theano.config.floatX),
    )

def print_sample_stats(samples, title):
    print('{} {} samples'.format(len(samples), title))
    samples_key_codes = [x[2] for x in samples]
    key_codes = list(set(samples_key_codes))
    for k in list(set(samples_key_codes)):
        print(' - Key code {} ({}) occurs {} times'.format(k, KEY_CODES[k], len([x for x in samples_key_codes if x == k])))

if __name__ == '__main__':
    config = Config(sys.argv[1])

    network = Network(config)

    if os.path.exists(config.samples_file):
        with np.load(config.samples_file) as npzfile:
            samples = npzfile['samples']
    else:
        samples = get_samples(config)


    print_sample_stats(samples, 'all')
    np.random.shuffle(samples)
    training, testing = divide_test_training_data(samples, config.training_ratio)
    print_sample_stats(training, 'training')
    print_sample_stats(testing, 'testing')

    stats = Stats(config)

    start_epoch = 0
    if len(sys.argv) > 2:
        # we are continuing at the epoch specified, load the network and model
        start_epoch = int(sys.argv[2])
        prefix, ending = os.path.splitext(config.model_file)
        network.load('{}-{:05d}{}'.format(prefix, start_epoch, ending))
        stats.load(config.epoch_stats_file)

    if 'NO_PLOT' not in os.environ:
        stats.start_live_plot()

    X_test, y_test = build_batch(config, testing)

    X_all, y_all = build_batch(config, training)

    if config.batch_size:
        bs = config.batch_size
        batch_count = len(X_all) // bs
        batches = [(X_all[i:i+bs], y_all[i:i+bs]) for i in range(0, batch_count * bs, bs)]

        lost_samples = len(X_all) - batch_count * bs
        lost_samples_ratio = lost_samples * 1.0 / len(X_all)
        if lost_samples_ratio > 0.2:
            print('WARNING! With current batch_size {} and sample count {} there will be {} batches and {} lost samples ({:.1f}%)'.format(
                bs, len(X_all), batch_count, lost_samples, lost_samples_ratio*100))
    else:
        batch_count = 1
        batches = [(X_all, y_all)]

    try:
        for epoch in range(start_epoch, config.epochs) if config.epochs else itertools.count(start_epoch):
            X, y = batches[epoch % batch_count]
            training_cost = float(network.train(X, y))

            test_result = network.test(X_test, y_test)

            stats.add(training_cost, *test_result)
            stats.print_row()

            if epoch % 250 == 0:
                prefix, ending = os.path.splitext(config.model_file)
                network.save('{}-{:05d}{}'.format(prefix, epoch, ending))

            if epoch % 2000 == 0:
                stats.save(config.epoch_stats_file)

    finally:
        stats.save(config.epoch_stats_file)
        network.save(config.model_file)

