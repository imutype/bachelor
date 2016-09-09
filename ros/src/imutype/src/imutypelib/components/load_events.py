from __future__ import print_function
import rospy, rosbag, sys
import os.path

def count_items(bag, topics):
    count = 0
    for topic in topics:
        for connection in bag._get_connections(topic):
            for chunk in bag._chunks:
                count += chunk.connection_counts.get(connection.id, 0)
    return count

def load_events(config, bag_file, target):
    topics = ['/imu_events', '/key_events']
    bag = rosbag.Bag(bag_file)
    count = count_items(bag, topics)

    for i, (topic, msg, time) in enumerate(bag.read_messages(topics=topics)):
        progress = i * 1.0 / count
        c = int(round(progress * 40))
        print('\r{:>6d}/{} [{}{}] {:3.0f}%  {}'.format(i, count, c * '#', (40 - c) * ' ', progress * 100, os.path.basename(bag_file)), end='')
        sys.stdout.flush()
        target.send((topic, msg))
    print('')

