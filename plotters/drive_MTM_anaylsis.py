
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


def compare_MTM_pos(t_start=-1, t_end=-1):

    t_start, t_end = compute_start_and_end_times(t_start, t_end)

    num_msgs = cur_bag.get_message_count("/dvrk/MTMR/position_cartesian_current")
    current = np.zeros(shape=(num_msgs, 3))

    num_msgs = cur_bag.get_message_count("/dvrk/MTMR/position_cartesian_desired")
    desired = np.zeros(shape=(num_msgs, 3))

    num_msgs = cur_bag.get_message_count("/dvrk/MTMR/set_position_goal_cartesian")
    goal = np.zeros(shape=(num_msgs, 3))

    idx = 0
    for top, msg, t in cur_bag.read_messages(topics=["/dvrk/MTMR/position_cartesian_current"], start_time=t_start, end_time=t_end):
        cur = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        current[idx, :] = cur
        idx = idx + 1

    idx = 0
    for top, msg, t in cur_bag.read_messages(topics=["/dvrk/MTMR/position_cartesian_desired"], start_time=t_start, end_time=t_end):
        cur = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        current[idx, :] = cur
        idx = idx + 1

    idx = 0
    for top, msg, t in cur_bag.read_messages(topics=["/dvrk/MTMR/set_position_goal_cartesian"], start_time=t_start, end_time=t_end):
        cur = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        current[idx, :] = cur
        idx = idx + 1

if __name__ == "__main__":


    file = ""

    cur_bag = rosbag.Bag(file)

    print cur_bag.get_end_time() - cur_bag.get_start_time()

    ctr = 0
    final_time = 0
    for topic, msg, time in cur_bag.read_messages(['/ambf/env/user_study_time']):
        ctr = ctr + 1
        final_time = time

