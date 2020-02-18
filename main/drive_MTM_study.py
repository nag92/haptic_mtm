from Tkinter import *
import rosbag
from datetime import datetime
import subprocess
import signal
import shlex

import rospy
from std_msgs.msg import Time
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose
import time

import os

import threading


class UserStudy:
    def __init__(self):

        rospy.init_node('user_study_data')
        self._time_pub = rospy.Publisher('/ambf/env/user_study_time', Time, queue_size=1)
        self._dvrk_on_pub = rospy.Publisher('/dvrk/console/power_on', Empty, queue_size=1)
        self._dvrk_off_pub = rospy.Publisher('/dvrk/console/power_off', Empty, queue_size=1)
        self._dvrk_home_pub = rospy.Publisher('/dvrk/console/home', Empty, queue_size=1)
        self._goal_pub = rospy.Publisher('"/dvrk/MTMR/set_position_goal_cartesian"', Pose, queue_size=1)
        self._time_msg = 0
        self._start_time = 0
        self._active = False
        self._time_pub_thread = 0
        self._my_bag = 0

        self._topic_names = ["/dvrk/MTMR/set_position_goal_cartesian",
                             "ambf/env/psm/baselink/State",
                             "ambf/env/psm/baselink/Command",
                             "/ambf/image_data/camera1/compressed",
                             "/ambf/env/World/State",
                             "/ambf/env/user_study/Base/State",
                             "/ambf/env/user_study/PuzzleRed/State",
                             "/ambf/env/user_study/PuzzleYellow/State",
                             "/ambf/env/camera1/State",
                             "/ambf/env/camera2/State",
                             "/ambf/env/simulated_device_1/MTML/State",
                             "/ambf/env/simulated_device_1/MTMR/State",
                             "/ambf/env/HandleLeft/State",
                             "/ambf/env/HandleRight/State",
                             "/ambf/env/FixedBase/State",
                             "/ambf/env/MovingBase/State",
                             "/dvrk/MTML/position_cartesian_current",
                             "/dvrk/MTMR/position_cartesian_current",
                             "/dvrk/footpedals/clutch",
                             "/dvrk/footpedals/coag",
                             "/dvrk/MTML/set_wrench_body",
                             "/dvrk/MTMR/set_wrench_body",
                             "/ambf/env/user_study_time"]

        self._topic_names_str = ""
        self._rosbag_filepath = 0
        self._rosbag_process = 0

        for name in self._topic_names:
            self._topic_names_str = self._topic_names_str + ' ' + name


    def call(self):
        if self._rosbag_filepath is 0:
            self._active = True
            self._start_time = rospy.Time.now()
            self._time_pub_thread = threading.Thread(target=self.time_pub_thread_func)
            self._time_pub_thread.start()
            print("Start Recording ROS Bag")
            date_time_str = str(datetime.now()).replace(' ', '_')
            self._rosbag_filepath = './user_study_data/' + str(e1.get()) + '_' + date_time_str
            command = "rosbag record -O" + ' ' + self._rosbag_filepath + self._topic_names_str
            print "Running Command", command
            command = shlex.split(command)
            self._rosbag_process = subprocess.Popen(command)
            pose = Pose()
            pose.position.x = float(x.get())
            pose.position.y = float(y.get())
            pose.position.z = float(z.get())
            self._goal_pub.publish(pose)
        else:
            print "Already recording a ROSBAG file, please save that first before starting a new record"

    def save(self):

        if self._rosbag_filepath is not 0:

            # self._active = False
            filepath = self._rosbag_filepath
            self._rosbag_filepath = 0

            node_prefix = "/record"
            # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
            list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
            list_output = list_cmd.stdout.read()
            retcode = list_cmd.wait()
            assert retcode == 0, "List command returned %d" % retcode
            for node_name in list_output.split("\n"):
                if node_name.startswith(node_prefix):
                    os.system("rosnode kill " + node_name)

            print("Saved As:", filepath, ".bag")
            self._active = False

        else:
            print("You should start recording first before trying to save")

    def time_pub_thread_func(self):

        while self._active:
            self._time_msg = rospy.Time.now() - self._start_time
            self._time_pub.publish(self._time_msg)
            time.sleep(0.05)

    def dvrk_power_on(self):
        self._dvrk_on_pub.publish(Empty())
        time.sleep(0.1)

    def dvrk_power_off(self):
        self._dvrk_off_pub.publish(Empty())
        time.sleep(0.1)

    def dvrk_home(self):
        self._dvrk_home_pub.publish(Empty())
        time.sleep(0.1)




study = UserStudy()

master = Tk()
master.title("AMBF USER STUDY 1")
width = 550
height = 600
master.geometry(str(width)+'x'+str(height))
Label(master, text='trial number').grid(row=0)
Label(master, text='X').grid(row=1)
Label(master, text='Y').grid(row=2)
Label(master, text='Z').grid(row=3)

e1 = Entry(master)
e1.grid(row=0, column=1)

x = Entry(master)
x.grid(row=1, column=1)

y = Entry(master)
y.grid(row=2, column=1)

z = Entry(master)
z.grid(row=3, column=1)


# Set Default Value



button_start = Button(master, text="Start Record", bg="green", fg="white", height=8, width=20, command=study.call)
button_stop = Button(master, text="Stop Record (SAVE)", bg="red", fg="white", height=8, width=20, command=study.save)
button_destroy = Button(master, text="Close App", bg="black", fg="white", height=8, width=20, command=master.destroy)


button_on = Button(master, text="DVRK ON", bg="green", fg="white", height=4, width=10, command=study.dvrk_power_on)
button_off = Button(master, text="DVRK OFF", bg="red", fg="white", height=4, width=10, command=study.dvrk_power_off)
button_home = Button(master, text="DVRK HOME", bg="purple", fg="white", height=4, width=10, command=study.dvrk_home)




button_on.grid(row=20, column=1)
button_off.grid(row=40, column=1)
button_home.grid(row=60, column=1)

button_start.grid(row=20, column=2)
button_stop.grid(row=40, column=2)
button_destroy.grid(row=60, column=2)

master.mainloop()
