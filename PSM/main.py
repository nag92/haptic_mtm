# Import the Client from ambf_client package
from ambf_client import Client
import time
import numpy as np

from PSM import ForwardKinematics, InverseKinematics
import draw_helix

if __name__ == '__main__':

    # Create a instance of the client
    _client = Client()

    # Connect the client which in turn creates callable objects from ROS topics
    # and initiates a shared pool of threads for bi-directional communication
    _client.connect()

    psm_handle = _client.get_obj_handle('psm/baselink')
    # yawlink(0), pitchbacklink(1), maininsertionlink(4), toolrolllink(5), toolpitchlink(6), toolgripper1link(7)
    # print(' ')
    # print('PSM Base Rotation as Quaternion:')
    # print(psm_handle.get_rot())
    # print(psm_handle.get_joint_pos(0))
    psm_children_names = psm_handle.get_children_names()  # Get a list of children names belonging to this obj
    print(psm_children_names)
    psm_handle.set_joint_effort(0, 0.5)
    q = np.array([psm_handle.get_joint_pos(0), psm_handle.get_joint_pos(1), psm_handle.get_joint_pos(4),
                  psm_handle.get_joint_pos(5), psm_handle.get_joint_pos(6), psm_handle.get_joint_pos(7)])
    print q
    F = ForwardKinematics(q)
    print "forward kinematics is ", F[-1]
    goal = np.array([1, 0, 0, -0.01, 0, 1, 0, -0.003, 0, 0, 1, -0.01, 0, 0, 0, 1]).reshape((4, 4))
    print "goal is ", goal
    IK = InverseKinematics(q, goal)
    print "inverse kinematics ", IK
    # Lastly to cleanup
    _client.clean_up()