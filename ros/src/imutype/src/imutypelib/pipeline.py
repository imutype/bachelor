from .utils import Collector
from .components import load_events, merge_events, preprocess, extract_samples

def get_samples(config):
    samples = []

    c = len(config.bag_files)
    print("Collecting samples from {} bag{}. This may take a while...".format(c, 's' if c != 1 else ''))

    for bag_file in config.bag_files:
        collector = Collector()
        load_events(config, bag_file, merge_events(config, preprocess(config, collector())))
        samples += extract_samples(config, collector.items)

    print("Done loading samples.")

    return samples
