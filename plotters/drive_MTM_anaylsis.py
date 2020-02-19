
import rosbag
import os
import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d



def compute_start_and_end_times( t_start=-1, t_end=-1):
    if t_start is -1:
        t_start = rospy.Time(cur_bag.get_start_time())

    if t_end is -1:
        t_end = rospy.Time(cur_bag.get_end_time())

    return t_start, t_end


def get_velocity_trajectory(topic_name, t_start=-1, t_end=-1):
    t_start, t_end = compute_start_and_end_times(t_start, t_end)

    num_msgs = cur_bag.get_message_count(topic_name)
    force_trajectory = np.zeros(shape=(num_msgs, 3))
    idx = 0
    for top, msg, t in cur_bag.read_messages(topics=[topic_name], start_time=t_start, end_time=t_end):
        cur = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        force_trajectory[idx, :] = cur
        idx = idx + 1

    return force_trajectory


def get_force_trajectory(topic_name, t_start=-1, t_end=-1):
    t_start, t_end = compute_start_and_end_times(t_start, t_end)

    num_msgs = cur_bag.get_message_count(topic_name)
    force_trajectory = np.zeros(shape=(num_msgs, 3))
    idx = 0
    for top, msg, t in cur_bag.read_messages(topics=[topic_name], start_time=t_start, end_time=t_end):
        cur = np.array([msg.userdata[0], msg.userdata[1], msg.userdata[2]])
        force_trajectory[idx, :] = cur
        idx = idx + 1

    return force_trajectory

def compare_MTM_pos(t_start=-1, t_end=-1):

    t_start, t_end = compute_start_and_end_times(t_start, t_end)

    num_msgs = cur_bag.get_message_count("/dvrk/MTMR/position_cartesian_current")
    current = np.zeros(shape=(num_msgs, 2))

    num_msgs = cur_bag.get_message_count("/dvrk/MTMR/position_cartesian_desired")
    desired = np.zeros(shape=(num_msgs, 2))

    num_msgs = cur_bag.get_message_count("/dvrk/MTMR/set_position_goal_cartesian")
    goal = np.zeros(shape=(num_msgs, 2))

    idx = 0
    for top, msg, t in cur_bag.read_messages(topics=["/dvrk/MTMR/position_cartesian_current"], start_time=t_start, end_time=t_end):
        cur = np.array([msg.pose.position.x, msg.pose.position.y])
        current[idx, :] = cur
        idx = idx + 1

    idx = 0
    for top, msg, t in cur_bag.read_messages(topics=["/dvrk/MTMR/position_cartesian_desired"], start_time=t_start, end_time=t_end):
        cur = np.array([msg.pose.position.x, msg.pose.position.y])
        desired[idx, :] = cur
        idx = idx + 1

    idx = 0
    for top, msg, t in cur_bag.read_messages(topics=["/dvrk/MTMR/set_position_goal_cartesian"], start_time=t_start, end_time=t_end):
        cur = np.array([msg.position.x, msg.position.y])
        goal[idx, :] = cur
        idx = idx + 1

    fig, ax = plt.subplots()
    ax.plot(current[:, 0], current[:, 1], 'r')
    ax.plot(desired[:, 0], desired[:, 1], 'k')
    ax.plot(goal[:, 0], goal[:, 1], 'b*')
    ax.set(xlabel='time (s)', ylabel='voltage (mV)',
           title='About as simple as it gets, folks')
    ax.grid()

    fig.savefig("test.png")
    plt.show()



if __name__ == "__main__":


    file = ""

    cur_bag = rosbag.Bag(file)

    print cur_bag.get_end_time() - cur_bag.get_start_time()

    ctr = 0
    final_time = 0
    for topic, msg, time in cur_bag.read_messages(['/ambf/env/user_study_time']):
        ctr = ctr + 1
        final_time = time

