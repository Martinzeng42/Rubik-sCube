#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from intera_interface import gripper as robot_gripper
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
import copy
import rospy
import geometry_msgs.msg
import argparse
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

# # Ada
# work_dist = 0.15
# down_grap_z_offset = 0.06 
# rotate_y_offset = 0.036 
# alice = 0

# Alice
work_dist = 0.15
down_grap_z_offset = 0.06 
rotate_y_offset = 0.042 
alice = 0.01

# add the constraint to prevent collision
def add_box_obstacle(size, name, pose):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size
        co.primitives = [box]
        co.primitive_poses = [pose.pose]
        planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)
        planning_scene_publisher.publish(co)

# execute plan
def exe_plan(wpose, group, velocity):
    waypoints = []
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(
                     waypoints,   
                     velocity,        
                     0.0)         
    group.execute(plan)

# Move gripper to position above the cube and facing down
def to_top(wpose, group):
    set_top_pos(wpose)
    set_top_ori(wpose)
    exe_plan(wpose, group, 0.005)


# Gripper position at the top of the cube
def set_top_pos(wpose):
    wpose.position.x = 0.720
    # kwargs['right_gripper'].open()
    wpose.position.y = 0.089
    wpose.position.z = -0.020

# Gripper initial orientation
def set_top_ori(wpose):
    wpose.orientation.x = 0.7071068
    wpose.orientation.y = 0.7071068
    wpose.orientation.z = 0.0
    wpose.orientation.w = 0.0

# Gripper facing left orientation
def set_left_ori(wpose):
    wpose.orientation.x = -0.5
    wpose.orientation.y = -0.5
    wpose.orientation.z = -0.5
    wpose.orientation.w = 0.5

# Gripper clockwise 90 orientation
def set_cw_ori(wpose):
    wpose.orientation.x = 1
    wpose.orientation.y = 0
    wpose.orientation.z = 0
    wpose.orientation.w = 0

# Gripper counter clockwise 90 orientation
def set_ccw_ori(wpose):
    wpose.orientation.x = 0
    wpose.orientation.y = 1
    wpose.orientation.z = 0
    wpose.orientation.w = 0

# Move gripper to the left position
def to_left(wpose, group):
    set_top_pos(wpose)
    set_left_ori(wpose)
    exe_plan(wpose, group, 0.005)

counter = 0
# Turn clockwise
def to_turn_clockwise(wpose, group, right_gripper, cube=None):
    global counter
    global alice
    go_down(wpose, group, down_grap_z_offset + alice)
    rospy.sleep(1.)
    right_gripper.close()
    rospy.sleep(0.5)
    set_cw_ori(wpose)
    exe_plan(wpose, group, 0.01)
    right_gripper.open()
    to_top_no_ori(wpose, group)
    to_top_ori(wpose,group)

    if not (cube is None):
        cube.turn_U_CW()
        counter += 1
        cube.render(flat=False).savefig('test{}.png'.format(counter), dpi=300)


# Turn counter clockwise
def to_turn_counter_clockwise(wpose, group, right_gripper, cube=None):
    global counter
    global alice
    go_down(wpose, group, down_grap_z_offset + alice)
    rospy.sleep(1.)
    right_gripper.close()
    rospy.sleep(0.5)
    set_ccw_ori(wpose)
    exe_plan(wpose, group, 0.01)
    right_gripper.open()
    to_top_no_ori(wpose, group)
    to_top_ori(wpose,group)

    if not (cube is None):
        cube.turn_U_CCW()
        counter += 1
        cube.render(flat=False).savefig('test{}.png'.format(counter), dpi=300)


# Roll the cube
def to_roll(wpose,  group, right_gripper, cube=None):
    global counter
    # rospy.sleep(2.)
    # go_down(wpose, group, down_grap_z_offset)
    # rospy.sleep(2.)
    # right_gripper.close()
    # rospy.sleep(0.5)
    # go_up(wpose, group, work_dist)
    # to_left(wpose, group)
    # go_left(wpose, group, rotate_y_offset)
    # rospy.sleep(1.)
    # go_down(wpose, group, 0.085)
    # rospy.sleep(2.)
    # go_down(wpose, group, rotate_y_offset - 0.015)
    # right_gripper.open()
    # to_top(wpose, group)

    go_down(wpose, group, down_grap_z_offset + 0.01)
    rospy.sleep(1.)
    right_gripper.close()
    rospy.sleep(0.5)
    go_up(wpose, group, work_dist)
    to_left(wpose, group)
    go_left(wpose, group, rotate_y_offset - 0.01)
    rospy.sleep(1.)
    go_down(wpose, group, 0.08) # 0.085
    input('Press [ Enter ]: ')
    go_down(wpose, group, rotate_y_offset - 0.015)
    right_gripper.open()
    to_top(wpose, group)

    if not (cube is None):
        cube.roll()
        counter += 1
        cube.render(flat=False).savefig('test{}.png'.format(counter), dpi=300)


# Set the gripper to the top but not change gripper orientation.
def to_top_no_ori(wpose, group):
    set_top_pos(wpose)
    exe_plan(wpose,group, 0.005)

# Set the gripper to the top and change gripper orientation.
def to_top_ori(wpose, group):
    set_top_ori(wpose)
    exe_plan(wpose,group, 0.005)

# Move gripper down by z
def go_down(wpose, group, z):
    wpose.position.z -= z
    exe_plan(wpose,group, 0.0008)

# Move gripper up by z
def go_up(wpose, group, z):
    wpose.position.z += z
    exe_plan(wpose,group, 0.005)

# Move gripper left by y
def go_left(wpose, group, y):
    wpose.position.y -= y
    exe_plan(wpose,group, 0.005)

# Turn the cube 180 degree
def to_turn_180(wpose,  group, right_gripper, cube=None):
    global counter
    to_turn_clockwise(wpose, group, right_gripper)
    to_turn_clockwise(wpose, group, right_gripper)

    if not (cube is None):
        cube.turn_U_180()
        counter += 1
        cube.render(flat=False).savefig('test{}.png'.format(counter), dpi=300)

# Lift up the cube and turn the cube clockwise 90 then put down. 
def switch_front_face(wpose, group, right_gripper, cube=None):
    global counter
    # put_dist_z = down_grap_z_offset + 0.015
    # go_down(wpose, group, down_grap_z_offset)
    # rospy.sleep(2.)
    # right_gripper.close()
    # rospy.sleep(1.)
    # go_up(wpose, group, work_dist)
    # set_ccw_ori(wpose)
    # exe_plan(wpose, group, 0.005)
    # go_down(wpose, group, put_dist_z)
    # input('Press [ Enter ] Going to Pick up Location: ')
    # rospy.sleep(2.)
    # go_down(wpose, group, work_dist - put_dist_z)
    # right_gripper.open()
    # to_top_no_ori(wpose, group)
    # to_top_ori(wpose,group)

    put_dist_z = work_dist - 0.015
    go_down(wpose, group, down_grap_z_offset)
    rospy.sleep(1.)
    right_gripper.close()
    rospy.sleep(0.5)
    go_up(wpose, group, work_dist)
    set_ccw_ori(wpose)
    exe_plan(wpose, group, 0.01)
    go_down(wpose, group, put_dist_z)
    input('Press [ Enter ]: ')
    go_down(wpose, group, 0.015)
    right_gripper.open()
    to_top_no_ori(wpose, group)
    to_top_ori(wpose,group)

    if not (cube is None):
        cube.switch_front_face()
        counter += 1
        cube.render(flat=False).savefig('test{}.png'.format(counter), dpi=300)

def lift_cube(wpose, group, right_gripper):
    go_down(wpose, group, down_grap_z_offset)
    rospy.sleep(1.)
    right_gripper.close()
    rospy.sleep(0.5)
    go_up(wpose, group, 0.03)

def rot_lifed_cube(wpose, group, right_gripper):
    set_ccw_ori(wpose)
    exe_plan(wpose, group, 0.005)
    drop_cube(wpose, group, right_gripper)
    right_gripper.open()
    to_top_no_ori(wpose, group)
    to_top_ori(wpose,group)

def drop_cube(wpose, group, right_gripper, cube=None):
    go_down(wpose, group, 0.01)
    input('Press [ Enter ]: ')
    go_down(wpose, group, 0.02)
    right_gripper.open()
    rospy.sleep(1.)
    to_top(wpose, group)

#make the right face facing down
def roll_n_times(n, **kwargs):
    for _ in range(n):
        to_roll(**kwargs)

def cube_config_str_to_arr(s):
    arr = np.zeros((6,3,3), dtype=np.int32)
    face_idx_dict = {
        'U': 0,
        'D': 1,
        'F': 2,
        'B': 3,
        'R': 4,
        'L': 5
    }
    face_order = ['U', 'R', 'F', 'D', 'L', 'B']

    i = 0
    for f in face_order:
        cur_str = s[i:i+9]
        cur = [face_idx_dict[tmp] for tmp in cur_str]
        cur = np.array(cur).reshape((3,3), order='F')

        #swap first and third col
        tmp = np.copy(cur[:, 0])
        cur[:, 0] = cur[:, 2]
        cur[:, 2] = tmp

        arr[face_idx_dict[f]] = cur

        i += 9
    return arr