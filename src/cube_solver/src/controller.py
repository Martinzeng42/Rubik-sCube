#!/usr/bin/env python

import magic_cube.cube
import numpy as np
import kociemba
import matplotlib.pyplot as plt
import time
import rospy
from std_msgs.msg import String

from magic_cube.cube import Cube
from controller_command_lib import *
# Define the callback method which is called whenever this node receives a 
# message on its subscribed topic. The received message is passed as the first
# argument to callback().

def execute_step(face, direction, **kwargs):
    #whatever that face is, make that face facing down
    if face == 'U':
        roll_n_times(2, **kwargs)
    elif face == 'L':
        roll_n_times(1, **kwargs)
    elif face == 'R':
        roll_n_times(3, **kwargs)
    elif face == 'F':
        switch_front_face(**kwargs)
        roll_n_times(3, **kwargs)
    elif face == 'B':
        switch_front_face(**kwargs)
        roll_n_times(1, **kwargs)

    # bottom face according to direction
    if direction == 1:
        to_turn_clockwise(**kwargs)
    elif direction == 2:
        to_turn_180(**kwargs)
    elif direction == 3 or direction == -1:
        to_turn_counter_clockwise(**kwargs)
        
def solve_cube(cube_config_data):
    cube_config = cube_config_data.data
    print(type(cube_config))
    cube_config_arr = cube_config_str_to_arr(cube_config)
    cube = Cube(3, init_colors=cube_config_arr, whiteplastic=False)
    cube.render(flat=False).savefig('test00.png', dpi=300)
    input('confirm render')
    solution_arr = kociemba.solve(cube_config).split(' ')

    kwargs = {
        'wpose': geometry_msgs.msg.Pose(),
        'group': MoveGroupCommander("right_arm"),
        'right_gripper': robot_gripper.Gripper('right_gripper'),
        'cube': cube
    }
    # kwargs = {
        # 'wpose': None,
        # 'group': None,
        # 'right_gripper': None,
        # 'cube': cube
    # }
    to_top(kwargs['wpose'], kwargs['group'])
    kwargs['right_gripper'].open()
    print(len(solution_arr))
    print(solution_arr)
    for step in solution_arr:
        ori_face = step[0]
        # find where that face is currently
        for k in cube.face_corr_dict:
            if cube.face_corr_dict[k] == ori_face:
                face = k
        if len(step) == 1:
            direction = 1
        elif step[1] == '\'':
            direction = 3
        elif step[1] == '2':
            direction = 2
        execute_step(face, direction, **kwargs)
        print(face, direction)

# Define the method which contains the node's main functionality
def listener():
    print('listening')
    # Create a new instance of the rospy.Subscriber object which we can use to
    # receive messages of type std_msgs/String from the topic /chatter_talk.
    # Whenever a new message is received, the method callback() will be called
    # with the received message as its first argument.
    rospy.Subscriber("cube_config", String, solve_cube)

    # Wait for messages to arrive on the subscribed topics, and exit the node
    # when it is killed with Ctrl+C
    rospy.spin()


# Python's syntax for a main() method
if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)

    listener()