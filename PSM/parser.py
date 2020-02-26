
import rosbag
import rospy
import numpy as np
import csv
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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
    steps = []
    fixed = []
    curr = data[0]
    count = 0
    steps.append(curr-curr)
    fixed.append(np.round(curr,3))
    for ii in xrange(0, len(data)-1):
        curr = np.round(data[ii],3)
        nxt = np.round(data[ii+1],3)
        interval = np.round(nxt-curr, 3)
        if abs(interval[0]) > 0 or abs(interval[1]) > 0 or abs(interval[2]) > 0:
            steps.append(interval)
            fixed.append(curr)

    return np.array(fixed), steps


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

    file = "/home/nathanielgoldfarb/Downloads/2020-02-25_14:00:07.940022.bag"
    topic = "/dvrk/PSM2/position_cartesian_current"
    file_name = "test"
    data = getData(file, topic)
    rnd, steps = make_steps(data)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot( rnd[:,0], rnd[:,1], rnd[:,2]   )
    plt.show()
    #make_file(rnd, steps, file_name)