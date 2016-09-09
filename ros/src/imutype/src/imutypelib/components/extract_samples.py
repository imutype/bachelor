import numpy as np
import random, math
from imutypelib.constants import KEY_CODES

def distance(num, numbers):
    return min([abs(i-num) for i in numbers])


def extract_samples(config, preprosessed_events):
    """
    Extracts samples with the same amount of pressed and not pressed keys.
    Returns a list of these samples. Each sample is a tuple, containing
    (time, data, key_code), where key_code is 0 for a sample where no key
    was pressed.
    """

    MIN_DISTANCE = config.negative_sample_min_distance
    BEFORE_PRESSED = int(round(config.sequence_length * config.sequence_ratio))
    AFTER_PRESSED = config.sequence_length - BEFORE_PRESSED

    preprosessed_events = list(preprosessed_events)

    samples = []
    used_event_indices = []
    for k, key_code in enumerate(config.key_codes):
        key_index = config.n_inputs + k

        for i, event in enumerate(preprosessed_events):
            if i > BEFORE_PRESSED \
                and i < len(preprosessed_events) - AFTER_PRESSED \
                and not preprosessed_events[i-1][1][key_index] and event[1][key_index]:

                samples.append((event[0], i, key_code))
                used_event_indices.append(i)

    print('Extracted {} positive samples'.format(len(samples)))
    desired_count = int(math.ceil(len(samples) / len(config.key_codes)))
    print('Desired negative sample count: {}'.format(desired_count))

    new_indices = [i for i in range(BEFORE_PRESSED, len(preprosessed_events)-AFTER_PRESSED) if distance(i, used_event_indices) > MIN_DISTANCE]
    random.shuffle(new_indices)
    new_indices = new_indices[:desired_count]

    if len(new_indices) < desired_count:
        print('warning! unbalanced data set, {} desired, {} actual negative samples'.format(desired_count, len(new_indices)))

    for i in new_indices:
        samples.append((preprosessed_events[i][0], i, 0))

    samples = map(
        lambda sample: (
            sample[0],
            np.array([event[1][:config.n_inputs] for event in preprosessed_events[sample[1] - BEFORE_PRESSED:sample[1] + AFTER_PRESSED]]),
            sample[2],
        ),
        samples,
    )

    print('Total of {} samples'.format(len(samples)))
    samples_key_codes = [x[2] for x in samples]
    key_codes = list(set(samples_key_codes))
    for k in list(set(samples_key_codes)):
        print(' - Key code {} ({}) occurs {} times'.format(k, KEY_CODES[k], len([x for x in samples_key_codes if x == k])))

    return samples
