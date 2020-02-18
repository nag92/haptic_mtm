#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String

pub = rospy.Publisher('/trajectroy', Pose, queue_size=1)


def callback(pose):


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


if __name__ == '__main__':
    listener()