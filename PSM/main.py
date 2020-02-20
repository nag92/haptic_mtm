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
    psm_joint_names = psm_handle.get_joint_names() # Get a list of children names belonging to this obj
    print(psm_joint_names)
    # psm_handle.set_joint_pos(1, 0.5)
    q = np.array([psm_handle.get_joint_pos(0), psm_handle.get_joint_pos(1), psm_handle.get_joint_pos(4),
                  psm_handle.get_joint_pos(5), psm_handle.get_joint_pos(6), psm_handle.get_joint_pos(7)])
    print q
    F = ForwardKinematics(q)
    print "forward kinematics is ", F[-1]
    # goal = np.array([1, 0, 0, -0.01, 0, 1, 0, -0.003, 0, 0, 1, -0.01, 0, 0, 0, 1]).reshape((4, 4))
    goal = np.array([-0.93575205, 0.35095179, 0.03465456, 0.01259456, -0.35120822, -0.93629631, -0.00141244, 0.01273318, 0.03195124, -0.01349266, 0.99939835, 0.00977501, 0., 0., 0., 1.]).reshape((4, 4))
    print "goal is ", goal
    IK = InverseKinematics(q, goal)
    print "inverse kinematics ", IK
    # forward
    # kinematics is [[-0.93575205  0.35095179  0.03465456  0.01259456]
    #                [-0.35120822 - 0.93629631 - 0.00141244  0.01273318]
    # [0.03195124 - 0.01349266
    # 0.99939835
    # 0.00977501]
    # [0.          0.          0.          1.]]
    # Lastly to cleanup
    _client.clean_up()