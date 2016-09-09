#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import print_function
import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import theano
import theano.tensor as T
import lasagne
import numpy as np
import imutype.msg as msg
import rospy

rospy.init_node('apply_net')

from imutypelib.config import Config
from imutypelib.network import Network
from imutypelib.components import merge_events, preprocess, extract_samples
from imutypelib.constants import KEY_CODES
from imutypelib.utils import coroutine, Collector

# todo: publish

config = Config(sys.argv[1])
network = Network(config)
network.load(config.model_file)

@coroutine
def apply(network, target):
    while True:
        sequence = (yield)
        if len(sequence) != config.sequence_length:
            continue
        inputs = np.array([[s[:config.n_inputs] for time, s in sequence]], dtype=theano.config.floatX)
        outputs = network.compute_network_output(inputs)[0]
        target.send(outputs)

@coroutine
def publisher():
    keyEventTopic = rospy.Publisher('/predicted_key_events', msg.KeyEvent, queue_size=1, tcp_nodelay=True)

    codes = [0] + config.key_codes
    was_pressed = [True] + [False for key_code in codes[1:]]

    while True:
        output = (yield)

        # Check for each output node that maps to a key whether this node's value is higher
        # than the threshold, i.e. the "no key was pressed" probability
        pi = np.argmax(output)
        if False: # threshold mode
            pressed = [pi == 0] + [p > output[0] for p in output[1:]]
        else: # argmax mode
            if output[pi] < 0.5: pi = 0
            pressed = [i == pi for i in range(len(codes))]

        # check for each key whether it was changed, publish new press/release event if so
        # if was_pressed != pressed:
        #     print('')
        print('\r', end='')
        for i, (was_key_pressed, key_pressed, key_code) in enumerate(zip(was_pressed, pressed, codes)):
            print(('\033[33m[{} {:.2f}]\033[0m' if key_pressed else ' {} {:.2f} ').format(KEY_CODES[key_code], output[i]), end='   ')
            if was_key_pressed != key_pressed:
                # print('Key {} ({}) was {}'.format(key_code, KEY_CODES[key_code], 'pressed' if key_pressed else 'released'))
                keyEventTopic.publish(rospy.Time.now(), key_code, key_pressed)
        sys.stdout.flush()

        was_pressed = pressed

if __name__ == '__main__':
    collector = Collector(max_count=config.sequence_length)

    preprocessor = preprocess(config, collector(apply(network, publisher())))
    pipeline = merge_events(config, preprocessor)

    def onImuEvent(event):
        try:
            pipeline.send(('/imu_events', event))
        except StopIteration as e:
            sys.exit(1)

    rospy.Subscriber('/imu_events', msg.ImuEvent, onImuEvent, queue_size=1, tcp_nodelay=True)
    print('\n'*20)
    rospy.spin()
