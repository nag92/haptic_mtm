#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import dvrk
import pickle
import numpy as np

pub = rospy.Publisher('/trajectroy', Pose, queue_size=1)


def callbacK(point):

    model_name = 'mtm'
    robot_name = 'MTMR'
    motor2dvrk_mtm = np.array([[1.0, 0, 0], [-1.0, 1.0, 0], [0.6697, -0.6697, 1.0]])

    # wait for a short period of time before recording data
    sampling_rate = 200
    speed = 0.4

    p = dvrk.mtm(robot_name)
    p.home()

    rospy.sleep(3)

    jonits_array = np.array([d for d in range(7)])

    q_start = np.deg2rad(-85)
    q_end = np.deg2rad(195)

    q_trajectory = np.linspace(q_start, q_end, num=int(sampling_rate * (q_end - q_start) / speed))
    states = np.zeros((q_trajectory.shape[0] * 2, 3))

    q = np.array([0., np.deg2rad(2.5), np.deg2rad(2.5), q_start, 0., 0., 0.])
    p.move_joint_some(q, jonits_array)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('MTM_traj', anonymous=True)

    rospy.Subscriber("/desired_pose", Pose, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def load_data(folder, name):
    model_file = folder + name + '.pkl'
    if os.path.exists(model_file):
        data = pickle.load(open(model_file, 'rb'))
        return data
    else:
        raise Exception("No {} can be found!".format(model_file))

if __name__ == '__main__':
    listener()