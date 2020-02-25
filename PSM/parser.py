
import rosbag
import rospy
import numpy as np
import csv
import matplotlib.pyplot as plt

def getData(bag, topic):
    """
    Extract a pos from the a topic
    :param bag: path to bag file
    :param topic: name of topic
    :return: the pos from that topic in a Nx3 array
    """
    cur_bag = rosbag.Bag(bag)

    t_start = rospy.Time(cur_bag.get_start_time())
    t_end = rospy.Time(cur_bag.get_end_time())

    num_msgs = cur_bag.get_message_count(topic)
    data = np.zeros(shape=(num_msgs, 3))
    idx = 0

    for top, msg, t in cur_bag.read_messages(topics=[topic],
                                             start_time=t_start,
                                             end_time=t_end):
        cur = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.y])
        data[idx, :] = cur
        idx = idx + 1

    return data


def make_steps(data):
    """
    Finds the steps between the data points
    :param data: numpy array
    :return: array steps between the data points
    """
    rnd = np.round(data, 3)
    steps = []
    curr = rnd[0]
    count = 0
    steps.append(curr-curr)
    for ii in xrange(0, len(rnd)-1):
        curr = rnd[ii]
        nxt = rnd[ii+1]
        interval = nxt-curr
        steps.append(np.round(interval,3))

    return rnd, steps


def make_file(data, steps, name):
    """
    Writes the data into a CSV file
    :param data: data nx3
    :param steps: steps betwen data Nx3
    :param name: name of CSV file
    :return: None
    """
    with open(name + '.csv', mode='w') as f:
        writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        writer.writerow(['X', 'X_step', 'Y', 'Y_step', 'Z', 'Z_step' ])

        for d, step in zip(data, steps):
            writer.writerow([ d[0], step[0], d[1], step[1], d[2], step[2]])


if __name__ == "__main__":

    file = "/home/nathaniel/Documents/2020-02-25_14:00:07.940022.bag"
    topic = "/dvrk/PSM2/position_cartesian_current"
    file_name = "test"
    data = getData(file, topic)
    x = data[:,0]
    rnd, steps = make_steps(data)
    make_file(rnd, steps, file_name)