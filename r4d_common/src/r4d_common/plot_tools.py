import os
import rosbag
import pandas as pd
from tf.transformations import euler_from_quaternion
from itertools import groupby
import matplotlib.pyplot as plt
import pprint

def get_data(s):
    data = {k: [[], []] for k in [x[0] for x in s]}

    for bag_file, by_bag_group in groupby(sorted(s, key=lambda x: x[1] + x[2]), lambda x: x[1]):
        by_bag_group = list(by_bag_group)
        bag = rosbag.Bag(os.path.expanduser(bag_file))

        # for this bag, build a mapping topic name -> [spec, spec]
        mapping = {x[2]: [] for x in by_bag_group}
        for topic, by_topic_group in groupby(by_bag_group, key=lambda x: x[2]):
            by_topic_group = list(by_topic_group)
            for spec in by_topic_group:
                mapping[topic].append(spec)

        print "{}\n".format(bag_file) + "="*20
        pprint.pprint(mapping)
        print

        for topic, msg, t in bag.read_messages(mapping.keys()):
            for spec in mapping[topic]:
                label, _, _, plot_fn = spec
                if hasattr(msg, 'header'):
                    data[label][0].append(msg.header.stamp.to_sec())
                else:
                    data[label][0].append(t)
                data[label][1].append(plot_fn(msg))

    return pd.DataFrame.from_dict({k: pd.Series(v[1], index=v[0]) for k, v in data.items()})

def simple_plot(s):
    df = get_data(s)
    df.plot(marker='.')
    plt.show()

#
# A couple of utility functions to extract data from common types of messages

odom_pos = lambda c: lambda m: getattr(m.pose.pose.position, c)
odom_vel = lambda c: lambda m: getattr(m.twist.twist.linear, c)
odom_quat = lambda c: lambda m: getattr(m.pose.pose.orientation, c)

def odom_euler(i):
    def f(m):
        q = [getattr(m.pose.pose.orientation, c) for c in 'xyzw']
        angles = euler_from_quaternion(q, axes='szyx')
        return angles[i]
    return f

def markers_count():
    def f(msg):
        return len(msg.tags)
    return f

def has_marker(id):
    def f(msg):
        this_marker = [marker for marker in msg.tags if marker.id == id]
        return len(this_marker)
    return f

def marker_pos(id, coord):
    def f(msg):
        this_marker = [marker for marker in msg.tags if marker.id == id]
        if this_marker:
            return getattr(this_marker[0].pose.position, coord)
        else:
            return None
    return f

def marker_quat(id, coord):
    def f(msg):
        this_marker = [marker for marker in msg.tags if marker.id == id]
        if this_marker:
            return getattr(this_marker[0].pose.orientation, coord)
        else:
            return None
    return f

acc_bias = lambda c: lambda m: getattr(m.linear_acceleration, c)
gyro_bias = lambda c: lambda m: getattr(m.angular_velocity, c)