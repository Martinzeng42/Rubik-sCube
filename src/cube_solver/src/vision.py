#!/usr/bin/env python

# Import the rospy package. For an import to work, it must be specified
# in both the package manifest AND the Python file in which it is used.
import rospy
import numpy as np
# Import the String message type from the /msg directory of the std_msgs package.
from std_msgs.msg import String
import geometry_msgs.msg
# from my_chatter.msg import TimestampString
# Define the method which contains the node's main functionality
from cube_solver.srv import RobotVision
from visionHelpers import *
import time

def publish_cube_config(cube_config):
    pub = rospy.Publisher('cube_config', String, queue_size=10)

    # make sure subscriber is subscribed before publishing:
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        rospy.loginfo('Connections: %d', connections)
        if connections > 0:
            pub.publish(String(cube_config))
            rospy.loginfo('Published config: {}'.format(cube_config))
            break
        rate.sleep()

def set_command(command, wpose):
    commands = RobotVision()
    commands.command = command
    commands.wpose = wpose
    return commands

def show_face(wpose, srv_proxy):
    command = set_command("show_cur_face", wpose)
    wpose = exe_command(command, srv_proxy)
    return wpose

def exe_command(command, srv_proxy):
    resp = srv_proxy(command.command, command.wpose)
    if not resp.success:
        raise rospy.ServiceException("Fail: command unsuccess.")
    return resp.wpose

def cali_manul(color_palette, colors):
    # rospy.wait_for_service('/robot_vision')
    try:
        # srv_proxy = rospy.ServiceProxy('/robot_vision', RobotVision)
        # wpose = geometry_msgs.msg.Pose()
        # command = set_command("init", wpose)
        # wpose = exe_command(command, srv_proxy)
        # record the horizontal faces first
        for i in range(4):
            # wpose = show_face(wpose, srv_proxy)
            input('Press [ Enter ] Going to Pick up Location: ')
            color_palette[colors[i]] = calibrete_color_a()
            # input('Press [ Enter ] Going to Pick up Location: ')
            # command = set_command("get_next_hori_face", wpose)
            # wpose = exe_command(command, srv_proxy)

        # perform a roll to get the bottom face 
        # command = set_command("get_bottom_face", wpose)
        # wpose = exe_command(command, srv_proxy)
        # wpose = show_face(wpose, srv_proxy)
        input('Press [ Enter ] Going to Pick up Location: ')
        color_palette[colors[4]] = calibrete_color_a()
        # command = set_command("drop_cube", wpose)
        # wpose = exe_command(command, srv_proxy)
        
        # perform double roll to get the top face 
        # command = set_command("get_top_face", wpose)
        # wpose = exe_command(command, srv_proxy)
        # wpose = show_face(wpose, srv_proxy)
        input('Press [ Enter ] Going to Pick up Location: ')
        color_palette[colors[5]] = calibrete_color_a()
        # command = set_command("drop_cube", wpose)
        # wpose = exe_command(command, srv_proxy)

        # perform a roll to get back to the starting position
        # command = set_command("reset", wpose)
        # wpose = exe_command(command, srv_proxy)
        return color_palette

    except rospy.ServiceException as e:
        rospy.loginfo(e)

def cali_bot(color_palette, colors):
    rospy.wait_for_service('/robot_vision')
    try:
        srv_proxy = rospy.ServiceProxy('/robot_vision', RobotVision)
        wpose = geometry_msgs.msg.Pose()
        command = set_command("init", wpose)
        wpose = exe_command(command, srv_proxy)
        # record the horizontal faces first
        for i in range(4):
            wpose = show_face(wpose, srv_proxy)
            color_palette[colors[i]] = calibrete_color_a()
            # print(color_palette)
            command = set_command("get_next_hori_face", wpose)
            wpose = exe_command(command, srv_proxy)

        # perform a roll to get the bottom face 
        command = set_command("get_bottom_face", wpose)
        wpose = exe_command(command, srv_proxy)
        wpose = show_face(wpose, srv_proxy)
        color_palette[colors[4]] = calibrete_color_a()
        command = set_command("drop_cube", wpose)
        wpose = exe_command(command, srv_proxy)
        
        # perform double roll to get the top face 
        command = set_command("get_top_face", wpose)
        wpose = exe_command(command, srv_proxy)
        wpose = show_face(wpose, srv_proxy)
        color_palette[colors[5]] = calibrete_color_a()
        command = set_command("drop_cube", wpose)
        wpose = exe_command(command, srv_proxy)

        # perform a roll to get back to the starting position
        command = set_command("reset", wpose)
        wpose = exe_command(command, srv_proxy)
        return color_palette

    except rospy.ServiceException as e:
        rospy.loginfo(e)


def robot_vision_client(color_palette):
    # send command to robot_vision service to tell the robot to go somewhere in order to take a photo
    rospy.wait_for_service('/robot_vision')
    try:
        srv_proxy = rospy.ServiceProxy('/robot_vision', RobotVision)
        wpose = geometry_msgs.msg.Pose()
        command = set_command("init", wpose)
        wpose = exe_command(command, srv_proxy)
        cube = []
        # record the horizontal faces first
        for i in range(4):
            wpose = show_face(wpose, srv_proxy)
            face = record_face(color_palette)
            input('Press [ Enter ] Going to Pick up Location: ')
            if face == None:
                print("Fail: record_face unsuccess.")
                return
            cube.append(face)
            command = set_command("get_next_hori_face", wpose)
            wpose = exe_command(command, srv_proxy)

        # perform a roll to get the bottom face 
        command = set_command("get_bottom_face", wpose)
        wpose = exe_command(command, srv_proxy)
        wpose = show_face(wpose, srv_proxy)
        face = record_face(color_palette)
        if face == None:
            print("Fail: record_face unsuccess.")
            return
        command = set_command("drop_cube", wpose)
        wpose = exe_command(command, srv_proxy)
        
        # perform double roll to get the top face 
        cube.append(face)
        command = set_command("get_top_face", wpose)
        wpose = exe_command(command, srv_proxy)
        wpose = show_face(wpose, srv_proxy)
        face = record_face(color_palette)
        if face == None:
            print("Fail: record_face unsuccess.")
            return
        command = set_command("drop_cube", wpose)
        wpose = exe_command(command, srv_proxy)
        cube.append(face)

        # perform a roll to get back to the starting position
        command = set_command("reset", wpose)
        wpose = exe_command(command, srv_proxy)
        return cube_to_config(cube)

    except rospy.ServiceException as e:
        rospy.loginfo(e)

    # take photo with camera
    #TODO


if __name__ == '__main__':
    rospy.init_node('vision', anonymous=True)
    try:
        colors = [
            'blue',
            'white',
            'green',
            'yellow',
            'red',
            'orange',
        ]
        # color_palette = {'yellow': [99, 199, 224],
        #  'orange': [62, 51, 63],
        #   # 'blue': [140, 87, 79],
        #   'blue': [255, 132, 0],
        #    'red': [41, 43, 153],
        #     'green': [89, 122, 101],
        #      'white': [202, 230, 247]}
        # color_palette = {
        # 'yellow': [204, 249, 252],
        #   'orange': [53, 43, 46],
        #    'blue': [168, 85, 39],
        #     'red': [0, 1, 166],
        #      'green': [181, 250, 227],
        #       'white': [251, 250, 252]}
        # color_palette = {
        #  'yellow': [167, 246, 252],
        #   'orange': [91, 87, 97],
        #    'blue': [209, 156, 142],
        #     'red': [50, 61, 165],
        #      'green': [29, 122, 0],
        #       'white': [251, 250, 252]}
        color_palette = {
         'yellow': [73, 170, 191],
         'orange': [47, 38, 43],
         'blue': [127, 74, 62],
         'red': [27, 33, 146],
         'green': [81, 112, 78],
         'white': [154, 161, 181]
        }

        # cali_bot(color_palette, colors)LULBBUB
        # print(color_palette)
        # TODO: code for getting images
        # cube_config = robot_vision_client(color_palette)

        # TODO: code for processing images. assuming the result will be stored in cube_config, a 6x3x3 array
        # cube_config = 'DRLUUBFBRBLURRLRUBLRDDFDLFUFUFFDBRDUBRUFLLFDDBFLUBLRBD'
        # cube_config = 'UUUUUUUUURRRRRRBBBFFFFFFRRRDDDDDDDDDLLLLLLFFFBBBBBBLLL'
        # cube_config = 'LLUUUBLLBUUFRRRUUFFFLFFFDDLRRFDDFRRDDLDDLDBLBRBBBBBRUU'
        cube_config = ''
        sides_arr = ['Up', 'Right', 'Front', 'Down', 'Left', 'Back']
        color_to_letter_dict = {
            'W': 'U',
            'O': 'R',
            'B': 'F',
            'Y': 'D',
            'R': 'L',
            'G': 'B',
        }
        for i in range(6):
            tmp = input('enter colors for {} :'.format(sides_arr[i]))
            converted = [color_to_letter_dict[k] for k in tmp]
            converted_str = ''.join(converted)
            cube_config += converted_str
        print('cube_config', cube_config)

        # cube_config = 'DRLUUBFBRBLURRLRUBLRDDFDLFUFUFFDBRDUBRUFLLFDDBFLUBLRBD'

        publish_cube_config(cube_config)
    except rospy.ROSInterruptException: 
        pass