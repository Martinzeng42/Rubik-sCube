#!/usr/bin/env python
import numpy as np
import rospy
from functools import partial
import sys
from controller_command_lib import *
from cube_solver.srv import RobotVision

def show_cur_face(wpose, group, right_gripper):
    lift_cube(wpose, group, right_gripper)

def get_next_hori_face(wpose, group, right_gripper):
    rot_lifed_cube(wpose, group, right_gripper)

def get_bottom_face(wpose, group, right_gripper):
    to_roll(wpose, group, right_gripper)

def drop_cube_a(wpose, group, right_gripper):
    drop_cube(wpose, group, right_gripper)

def get_top_face(wpose, group, right_gripper):
    to_roll(wpose, group, right_gripper)
    to_roll(wpose, group, right_gripper)

def reset(wpose, group, right_gripper):
    to_roll(wpose, group, right_gripper)

def init(wpose, group, right_gripper):
    right_gripper.open()
    to_top(wpose, group)

def callback(request):
    command = request.command
    wpose = request.wpose
    print('command: ', command)
    # TODO move the robot based on command, using functions from controller_command_lib
    group = MoveGroupCommander("right_arm")
    right_gripper = robot_gripper.Gripper('right_gripper')
    if (command == 'init'):
        init(wpose, group, right_gripper)
    if (command == 'show_cur_face'):
        show_cur_face(wpose, group, right_gripper)
    if (command == 'get_next_hori_face'):
        get_next_hori_face(wpose, group, right_gripper)
    if (command == 'get_bottom_face'):
        get_bottom_face(wpose, group, right_gripper)
    if (command == 'get_top_face'):
        get_top_face(wpose, group, right_gripper)
    if (command == 'reset'):
        reset(wpose, group, right_gripper)
    if (command == 'drop_cube'):
        drop_cube_a(wpose, group, right_gripper)
    return {'success' : True, 'wpose' : wpose}
def server():
    # Initialize the server node for turtle1
    rospy.init_node('controller_vision_server')
    # Register service
    rospy.Service(
        '/robot_vision',  # Service name
        RobotVision,  # Service type
        callback  # Service callback
    )
    rospy.loginfo('Running controller vision server...')
    rospy.spin() # Spin the node until Ctrl-C


if __name__ == '__main__':
    server()