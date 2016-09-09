from enum import Enum
import yaml
import rospy
import os, sys
import glob

default_file = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "config", "default.yaml"))

class NetworkType(Enum):
    cnn2d = "cnn2d"
    lstm = 'lstm'
    simple_recurrent = 'simple_recurrent'

class CostFunction(Enum):
    apm = 'apm'
    mse = 'mse'
    mpse = 'mpse'
    binary_cross_entropy = 'binary_cross_entropy'
    negative_f_score = 'negative_f_score'
    weighted_errors = 'weighted_errors'

class SamplingMode(int, Enum):
    full = 10
    quat = 4
    angles = 2
    quat_accel = 7

def find_file(p):
    if os.path.isfile(p):
        return p
    elif os.path.isdir(p):
        return os.path.join(p, 'config.yaml')
    else:
        expfile = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "experiments", p, "config.yaml"))
        if os.path.isfile(expfile):
            return expfile
        else:
            raise Exception("Config file or experiment directory not found: {}".format(p))

class Config(object):
    def __init__(self, *paths):
        self.files = [find_file(path) for path in paths] + [default_file]
        self.values = [yaml.load(open(filename)) for filename in self.files]

        # store file paths
        self.path = os.path.abspath(os.path.dirname(self.files[0]))

        bag_file = self.get_value('bag_file', str, '')
        if bag_file:
            print('Deprecated config entry bag_file, use bag_files instead.')
            self.bag_files = [bag_file]
        else:
            auto_bag_files = glob.glob(os.path.join(self.path, '*.bag'))
            self.bag_files = self.get_value('bag_files', list, auto_bag_files)

        self.training_file = os.path.join(self.path, 'training.npz')
        self.model_file = os.path.join(self.path, 'model.npz')
        self.epoch_stats_file = os.path.join(self.path, 'epoch_stats.npz')
        self.samples_file = os.path.join(self.path, 'samples.npz')
        self.samples_plot_file = os.path.join(self.path, 'samples.png')

        # fetch values
        self.live_mode = self.get_value('live_mode', bool)
        self.learn_mode = self.get_value('learn_mode', bool)

        self.imu_ids = self.get_value('imu_ids', list)
        self.key_codes = self.get_value('key_codes', list)
        self.training_ratio = self.get_value('training_ratio', float)
        self.learning_rate = self.get_value('learning_rate', float)
        self.epochs = self.get_value('epochs', lambda x: x or None)
        self.output_threshold = self.get_value('output_threshold', float)
        self.sequence_length = self.get_value('sequence_length', int)
        self.sequence_ratio = self.get_value('sequence_ratio', float)
        self.network_type = self.get_value('network_type', NetworkType)
        self.cost_function = self.get_value('cost_function', CostFunction)
        self.sampling_mode = self.get_value('sampling_mode', lambda x: SamplingMode.__members__[x])
        self.n_hidden = self.get_value('n_hidden', int)
        self.batch_size = self.get_value('batch_size', int)
        self.sampling_rate = self.get_value('sampling_rate', int)
        self.negative_sample_min_distance = self.get_value('negative_sample_min_distance', int)

        self.base_imu_relative_average_rate = self.get_value('base_imu_relative_average_rate', float, 0.2)
        self.finger_imu_relative_average_rate = self.get_value('finger_imu_relative_average_rate', float, 0)

        # confolutional approach
        self.convolution_filter_count = self.get_value('convolution_filter_count', int)
        self.convolution_filter_size = self.get_value('convolution_filter_size', list)
        self.convolution_iterations = self.get_value('convolution_iterations', int)
        self.convolution_dense_layer_units = self.get_value('convolution_dense_layer_units', list)
        self.convolution_deep_filters = self.get_value('convolution_deep_filters', bool)

    # auto-generate some properties
    @property
    def n_inputs(self):
        return self.sampling_mode * len(self.imu_ids)

    @property
    def n_outputs(self):
        return (1 if self.network_type == NetworkType.cnn2d else 0) + len(self.key_codes)

    def get_value(self, name, valueType, default_value=None):
        for values in self.values:
            if name in values:
                return valueType(values[name])

        if default_value is not None:
            return default_value
        raise Exception("Config value {} not defined.".format(name))
