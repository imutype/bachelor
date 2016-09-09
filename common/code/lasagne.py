l_current = l_in = lasagne.layers.InputLayer(
        (None, SEQUENCE_LENGTH, N_INPUTS))

for x in range(CONVOLUTION_LAYER_PAIRS):
    l_current = lasagne.layers.Conv2DLayer(
            l_current, FILTER_COUNT, FILTER_SIZE)

    l_current = lasagne.layers.MaxPool2DLayer(
            l_current, POOLING_SIZE)

l_out = lasagne.layers.DenseLayer(l_current, N_OUTPUTS)
