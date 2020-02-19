# Import the Client from ambf_comm package
# You might have to do: pip install gym

from ambf_client import Client
import time

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



def temp_controller():
    # Create a instance of the client
    _client = Client()

    # Connect the client which in turn creates callable objects from ROS topics
    # and initiates a shared pool of threads for bi-directional communication
    _client.connect()
    print('\n\n----')
    raw_input("We can see what objects the client has found. Press Enter to continue...")
    # You can print the names of objects found. We should see all the links found
    print(_client.get_obj_names())

    # Lets get a handle to PSM and ECM, as we can see in the printed
    # object names, 'ecm/baselink' and 'psm/baselink' should exist
    ecm_handle = _client.get_obj_handle('ecm/toollink')
    # Let's sleep for a very brief moment to give the internal callbacks
    # to sync up new data from the running simulator
    time.sleep(0.2)

    print('\n\n----')
    raw_input("Let's Get Some Pose Info. Press Enter to continue...")
    # Not we can print the pos and rotation of object in the World Frame
    print('ECM Base Pos:')
    print(ecm_handle.get_pos())


    print('\n\n----')
    raw_input("Let's get Joints and Children Info. Press Enter to continue...")
    # We can get the number of children and joints connected to each object as
    ecm_num_joints = ecm_handle.get_num_joints()  # Get the number of joints of this object
    print('Number of Joints in ECM:')
    print(ecm_num_joints)

    print(' ')
    print('Name of PSM\'s children:')

    print('\n\n----')
    raw_input("Control ECMs joint positions. Press Enter to continue...")
    # Now let's control some joints
    # The 1st joint, which the ECM Yaw
    ecm_handle.set_joint_pos(0, 0)
    # The 2nd joint, which is the ECM Pitch


    print('\n\n----')
    raw_input("Mixed Pos and Effort control of PSM\'s joints. Press Enter to continue...")

    print('\n\n----')
    raw_input("Set force on MTM's Wrist Yaw link for 5 secs. Press Enter to continue...")
    # Let's directly control the forces and torques on the mtmWristYaw Link
    # Notice that these are in the world frame. Another important thing to notice
    # is that unlike position control, forces control requires a continuous update
    # to meet a watchdog timing condition otherwise the forces are reset Null. This
    # is purely for safety reasons to prevent unchecked forces in case of malfunctioning
    # python client code


    print('\n\n----')
    raw_input("Let's clean up. Press Enter to continue...")
    # Lastly to cleanup
    _client.clean_up()

def controller():
    # Create a instance of the client
    _client = Client()

    _client.connect()

    # You can print the names of objects found
    print(_client.get_obj_names())


    tool = _client.get_obj_handle('ecm/remotecenterlink')
    body = _client.get_obj_handle('ecm/baselink')
    body.set_joint_pos(0,0)
    print body.get_children_names()
    pos = tool.get_pos()
    y = pos.y
    while 1:
        pos = tool.get_pos()
        x = pos.x
        print pos
        tool.set_pos(x+.1, 0.0, 0)
        time.sleep(0.1)

def make_helix(s, r, offset ):

    t = np.linspace(s, 4 * np.pi, 100)
    x = r*np.cos(t) - r
    z = r*np.sin(t)
    y = t - offset
    return x,y,z

def draw_simple_helix(x,y, z):

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(x,y,z)
    plt.show()


if __name__ == "__main__":
    #x, y, z = make_helix(0.1, 4)
    #draw_simple_helix(x, y, z)
    controller()