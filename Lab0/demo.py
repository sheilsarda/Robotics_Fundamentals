#!/usr/bin/python2
from time import sleep
import numpy as np
import rospy
import sys

from sys import path
from os import getcwd
path.append(getcwd() + "/../Core")

from arm_controller import ArmController

q = [.5,-0.5,-0.5,-0.1,1.2,0];

if __name__=='__main__':

    # set up ROS interface
    con = ArmController()
    sleep(1)
    rospy.loginfo("Setup complete !")

    # send command via controller
    con.set_state(q)
    sleep(3)
    rospy.loginfo("The current state is: ")
    print(con.get_state())

    # shut down ROS interface
    con.stop()
