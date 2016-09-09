import theano
import theano.tensor as T
import lasagne
import numpy as np

from .config import CostFunction, NetworkType
from .constants import KEY_CODES

class cached_property(object):
    def __init__(self, factory):
        self._attr_name = factory.__name__
        self._factory = factory

    def __get__(self, instance, owner):
        attr = self._factory(instance)
        setattr(instance, self._attr_name, attr)
        return attr

def create_boolean_non_linearity(threshold):
    def boolean_non_linearity(x):
        return T.switch(x > threshold, 1, 0)
    return boolean_non_linearity

def get_padding(n):
    nextPower2 = 1
    while nextPower2 < n:
        nextPower2 = nextPower2 * 2
    return (nextPower2 - n) / 2 + 1

class Network(object):
    def __init__(self, config):
        self.config = config

        self.training_inputs = None
        self.training_outputs = None
        self.testing_inputs = None
        self.testing_outputs = None

        self.batch_size = T.scalar('batch_size', dtype='int32')

        self.l_in = lasagne.layers.InputLayer(shape=(None, config.sequence_length, config.n_inputs))

        if self.config.network_type == NetworkType.lstm:
            self.target_values = T.vector('target_values')
            self.l_recurrent = lasagne.layers.LSTMLayer(self.l_in, config.n_hidden)
            self.l_out = lasagne.layers.DenseLayer(self.l_recurrent, num_units=config.n_outputs)

        elif self.config.network_type == NetworkType.simple_recurrent:
            self.target_values = T.vector('target_values')
            self.l_recurrent = lasagne.layers.RecurrentLayer(self.l_in, config.n_hidden)
            self.l_out = lasagne.layers.DenseLayer(self.l_recurrent, num_units=config.n_outputs)

        elif self.config.network_type == NetworkType.cnn2d:
            self.target_values = T.matrix('target_values')

            current_input_layer = self.l_in

            # current_input_layer = lasagne.layers.ReshapeLayer(
            #     current_input_layer, (self.batch_size, 1) + current_input_layer.output_shape[-2:],
            # )
            if config.convolution_deep_filters:
                current_input_layer = lasagne.layers.ReshapeLayer(
                    self.l_in,
                    (self.batch_size, 1, config.sequence_length, config.n_inputs),
                )

            for i in range(config.convolution_iterations):
                if not config.convolution_deep_filters:
                    current_input_layer = lasagne.layers.ReshapeLayer(
                        current_input_layer,
                        (self.batch_size * (50**i), 1) + current_input_layer.output_shape[-2:],
                    )
                    print('shape after reshape {}:     {}'.format(i, str(current_input_layer.output_shape)))

                l_convolution = lasagne.layers.Conv2DLayer(
                    current_input_layer,
                    config.convolution_filter_count,
                    config.convolution_filter_size,
                    pad='same',
                )

                # create max-pooling layer
                l_pool = lasagne.layers.MaxPool2DLayer(l_convolution, 2)

                print('shape after convolution {}: {}'.format(i, str(l_convolution.output_shape)))
                print('shape after pooling {}:     {}'.format(i, str(l_pool.output_shape)))

                current_input_layer = l_pool

            print('shape after stage 1:       {}'.format(str(current_input_layer.output_shape)))

            if not config.convolution_deep_filters:
                c = current_input_layer.output_shape[-2] * current_input_layer.output_shape[-1]
                f = 50 ** config.convolution_iterations
                current_input_layer = lasagne.layers.ReshapeLayer(
                    current_input_layer,
                    shape=(self.batch_size, f * c),
                )

                print('shape before stage 2:      {}'.format(str(current_input_layer.output_shape)))

            for i, num_units in enumerate(config.convolution_dense_layer_units):
                dense = lasagne.layers.DenseLayer(
                    current_input_layer,
                    num_units=num_units,
                )

                current_input_layer = dense
                print('shape after dense {}:       {}'.format(i, str(current_input_layer.output_shape)))

            self.l_out = lasagne.layers.DenseLayer(
                current_input_layer,
                num_units=config.n_outputs,
            )

            print('shape after output:        {}'.format(str(self.l_out.output_shape)))


        # self.l_out = lasagne.layers.NonlinearityLayer(self.l_dense, lasagne.nonlinearities.tanh)

    def set_datasets(self, training_inputs, training_outputs, testing_inputs, testing_outputs):
        self.training_inputs = training_inputs
        self.training_outputs = training_outputs
        self.testing_inputs = testing_inputs
        self.testing_outputs = testing_outputs

    @cached_property
    def compute_output(self):
        fn = theano.function([self.l_in.input_var, self.batch_size], self.output)
        return lambda X: fn(X, len(X))

    @cached_property
    def compute_network_output(self):
        network_output = lasagne.layers.get_output(self.l_out)
        fn = theano.function([self.l_in.input_var, self.batch_size], network_output)
        return lambda X: fn(X, len(X))

    @cached_property
    def output(self):
        network_output = lasagne.layers.get_output(self.l_out)

        if self.config.network_type == NetworkType.cnn2d:
            return T.argmax(network_output, axis=1)
        else:
            boolean_non_linearity = create_boolean_non_linearity(self.config.output_threshold)
            return boolean_non_linearity(network_output)

    @cached_property
    def cost(self):
        predicted_values = lasagne.layers.get_output(self.l_out)

        if self.training_outputs:
            N = self.training_outputs.shape[0]
            NP = T.sum(self.target_values)
            NN = N - NP

            TP = T.sum(self.target_values * predicted_values)
            TN = T.sum((1 - self.target_values) * (1 - predicted_values))
            FP = T.sum((1 - self.target_values) * predicted_values)
            FN = T.sum((1 - predicted_values) * self.target_values)

            pressed_proportion = (np.sum(self.training_outputs, axis=0) / N).reshape((self.training_outputs.shape[1],))

        # TODO: implement more cost functions and see how they perform

        # Proportional mean squared error
        if self.config.cost_function == CostFunction.mpse:
            return T.mean(((predicted_values - self.target_values) ** 2) * \
                    (self.target_values * (1 - pressed_proportion) + (1 - self.target_values) * pressed_proportion))

        # mean squared error
        elif self.config.cost_function == CostFunction.mse:
            return T.mean((predicted_values - self.target_values)**2)

        # Binary cross-entropy (does not work for some reason)
        elif self.config.cost_function == CostFunction.binary_cross_entropy:
            # TODO: try lasagne.objectives
            return T.mean(T.nnet.binary_crossentropy(
                T.nnet.softmax(predicted_values),
                self.target_values,
            ))

        # Negative F-Score
        elif self.config.cost_function == CostFunction.negative_f_score:
            # T.switch is a ternary operator (first: if, second: then, third: else) to avoid division by zero
            precision = T.switch(T.eq(TP + FP, 0), 0, TP / (TP + FP))
            recall = T.switch(T.eq(TP + FN, 0), 0, TP / (TP + FN))
            f_score = T.switch(T.eq(precision + recall, 0), 0, 2 * ((precision * recall) / (precision + recall)))
            return 1 - f_score


        # Weighted errors: include FN- and FP-rate based on positive_proportion and its inverse
        elif self.config.cost_function == CostFunction.weighted_errors:
            return T.sum(FP/N * (1 - pressed_proportion) + FN/N * pressed_proportion)

        elif self.config.cost_function == CostFunction.apm:
            return TP / T.nnet.softplus(NP) \
                + TN / T.nnet.softplus(NN) \
                + TP / T.nnet.softplus(TP + NN - TN)

    @cached_property
    def train(self):
        all_params = lasagne.layers.get_all_params(self.l_out)
        updates = lasagne.updates.adagrad(self.cost, all_params, self.config.learning_rate)

        fn = theano.function([self.l_in.input_var, self.target_values, self.batch_size], self.cost, updates=updates)
        return lambda X, y: fn(X, y, len(X))

    @cached_property
    def compute_cost(self):
        fn = theano.function([self.l_in.input_var, self.target_values, self.batch_size], self.cost)
        return lambda X, y: fn(X, y, len(X))


    def load(self, filename=None):
        print('loading' + str(filename or self.config.model_file))
        with np.load(filename or self.config.model_file) as npzfile:
            param_values = [npzfile[f] for f in sorted(npzfile.files, key=lambda x: int(x[4:]))]
        lasagne.layers.set_all_param_values(self.l_out, param_values)

    def save(self, filename=None):
        all_param_values = lasagne.layers.get_all_param_values(self.l_out)
        np.savez(filename or self.config.model_file, *all_param_values)

    def test(self, X, y_true):
        testing_cost = float(self.compute_cost(X, y_true))
        predicted = self.compute_output(X)

        if self.config.network_type == NetworkType.cnn2d:
            actual = np.argmax(y_true, axis=1)
        else:
            actual = y_true

        return actual, predicted, testing_cost
