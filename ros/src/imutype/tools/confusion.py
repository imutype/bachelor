#!/usr/bin/env python2
from __future__ import print_function
import os, sys, random
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

random.seed(0) # let's have some deterministic shuffling

import numpy as np
import theano
import itertools
import sklearn
from imutypelib.config import Config
from imutypelib.network import Network
from imutypelib.stats import Stats
from imutypelib.pipeline import get_samples


def divide_test_training_data(data, ratio):
    # data_split = [training, test]
    s = int(ratio * len(data))
    data_split = (data[:s], data[s:])
    return data_split

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


if __name__ == '__main__':
    config = Config(sys.argv[1])
    if len(sys.argv) > 2:
        config.model_file = config.model_file.replace('.npz', '-{}.npz'.format(sys.argv[2]))

    network = Network(config)
    network.load()

    if os.path.exists(config.samples_file):
        with np.load(config.samples_file) as npzfile:
            samples = npzfile['samples']
    else:
        samples = get_samples(config)

    np.random.shuffle(samples)
    training, testing = divide_test_training_data(samples, config.training_ratio)

    stats = Stats(config)

    X_test, y_test = build_batch(config, samples)

    actual, predicted, _ = network.test(X_test, y_test)
    print(sklearn.metrics.confusion_matrix(actual, predicted))
