import rospy
from imutypelib.utils import coroutine
import tf.transformations as t
import imutype.msg as msg


def lerp(a, b, f):
    return a + (b - a) * f

def reverse_lerp(left, right, middle):
    return (middle - left) / (right - left)

def interpolate_imu_events(a, b, time):
    if a == None:
        return b.data.accel + b.data.gyro + b.data.quat
    if b == None:
        return a.data.accel + a.data.gyro + a.data.quat
    # if a and b are None it will crash
    factor = reverse_lerp(a.time.to_sec(), b.time.to_sec(), time.to_sec())

    aa = a.data.accel + a.data.gyro
    bb = b.data.accel + b.data.gyro

    return list(map(lambda x, y: lerp(x, y, factor), aa, bb)) + \
        list(t.quaternion_slerp(a.data.quat, b.data.quat, factor))

def find_before_after(events, time):
    if not events:
        return None, None
    for i, event in enumerate(events):
        if event.time > time:
            if i == 0:
                return None, event
            return events[i-1], event
    return events[-1], None

def remove_all_previous_except_one(events, time):
    for i, event in enumerate(events):
        if event.time > time:
            return events[i-1:]
    return events


def extract_imu_values(config, imu_events, time):
    values = []
    for imu_id in config.imu_ids:
        before, after = find_before_after(imu_events[imu_id], time)
        values += interpolate_imu_events(before,after,time)
    return values

def extract_key_values(config, key_events, time):
    values = []
    for key_code in config.key_codes:
        before, after = find_before_after(key_events[key_code], time)
        if before:
            values += [before.pressed]
        elif after:
            values += [not after.pressed]
        else:
            values += [False]
    return values

@coroutine
def merge_events(config, target):
    interval = rospy.Duration.from_sec(1.0/config.sampling_rate)
    imu_events = {imu_id: [] for imu_id in config.imu_ids}

    key_events = {key_code: [] for key_code in config.key_codes}

    next_event_time = None

    while True:
        topic, raw_event = (yield)

        if next_event_time == None:
            next_event_time = raw_event.time + interval

        if topic == '/imu_events':
            if raw_event.imuId in config.imu_ids:
                imu_events[raw_event.imuId].append(raw_event)
        elif topic == '/key_events':
            if raw_event.code in config.key_codes:
                key_events[raw_event.code].append(raw_event)

        all_imus_have_at_least_one_event_past_next_event_time = all(map(
            lambda events: events and (events[-1].time > next_event_time),
            imu_events.values(),
        ))

        if all_imus_have_at_least_one_event_past_next_event_time:
            target.send((next_event_time, extract_imu_values(config, imu_events, next_event_time) + extract_key_values(config, key_events, next_event_time)))

            for k, v in imu_events.items():
                imu_events[k] = remove_all_previous_except_one(v, next_event_time)

            for k, v in key_events.items():
                key_events[k] = remove_all_previous_except_one(v, next_event_time)

            next_event_time += interval


