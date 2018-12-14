#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import copy

import rospy
import rospkg
import rosbag
import matplotlib.pyplot as plt
import matplotlib.figure

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)
from sensor_msgs.msg import (
    JointState
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from tf.transformations import *
import baxter_interface
from util.knowledge_base import KnowledgeBase
from action_primitive_variation.srv import *

actionToVary = None 
gripper = None
obj = None
button = None

block_bag = rosbag.Bag('block.bag' ,'w')
leftButton_bag = rosbag.Bag('leftButton.bag' ,'w')
rightButton_bag = rosbag.Bag('rightButton.bag' ,'w')
leftGripper_bag = rosbag.Bag('leftGripper.bag' ,'w')
rightGripper_bag = rosbag.Bag('rightGripper.bag' ,'w')
jointState_bag = rosbag.Bag('jointState.bag' ,'w')

BlockPose = None
shouldRecord = False

AP_names = ['press_button', 'obtain_object']
AP_services = ['press_button_srv', 'obtain_object_srv']
AP_srvs = [PressButtonSrv, ObtainObjectSrv]
KB = KnowledgeBase(AP_names, AP_services, AP_srvs)

def handle_APV(req):
    global gripper
    global actionToVary
    global obj
    global button
    global shouldRecord
    
    params = []
    actionToVary = req.actionName
    gripper = req.gripper
    if gripper is not '': 
        params.append(gripper)
    obj = req.obj
    if obj is not '': 
        params.append(obj)
    button = req.button
    if button is not '': 
        params.append(button)

    # open the bags in case they were closed
    global leftButton_bag 
    global rightButton_bag 
    global block_bag 
    global leftGripper_bag 
    global rightGripper_bag
    global jointState_bag

    block_bag = rosbag.Bag('block.bag' ,'w')
    leftButton_bag = rosbag.Bag('leftButton.bag' ,'w')
    rightButton_bag = rosbag.Bag('rightButton.bag' ,'w')
    leftGripper_bag = rosbag.Bag('leftGripper.bag' ,'w')
    rightGripper_bag = rosbag.Bag('rightGripper.bag' ,'w')
    jointState_bag = rosbag.Bag('jointState.bag' ,'w')

    shouldRecord = True
    execute_action(actionToVary, params)
    shouldRecord = False
  
    visualize_ROSbag_data()

    closeBags()

    return APVSrvResponse(1)


############### START: Call back functions that check to see if ROSbag should be being recorded
def handle_jointStates(data):
    global shouldRecord
    if shouldRecord is not False:
        try:
            jointState_bag.write('robot/joint_states', data)
        finally:
            pass


def handle_block(data):
    global shouldRecord
    if shouldRecord is not False:
        try:
            block_bag.write('block_pose', data)
        finally:
            pass

def handle_buttonLeft(data):
    global shouldRecord
    if shouldRecord is not False:
        try:
            leftButton_bag.write('left_button_pose', data)
        finally:
            pass

def handle_buttonRight(data):
    global shouldRecord
    if shouldRecord is not False:
        try:
            rightButton_bag.write('right_button_pose', data)
        finally:
            pass

def handle_gripperLeft(data):
    global shouldRecord
    if shouldRecord is not False:
        try:
            leftGripper_bag.write('left_gripper_pose', data)
        finally:
            pass

def handle_gripperRight(data):
    global shouldRecord
    if shouldRecord is not False:
        try:
            rightGripper_bag.write('right_gripper_pose', data)
        finally:
            pass


############### END: Call back functions that check to see if ROSbag should be being recorded

############### START: ROSbag handling
def visualize_ROSbag_data():

    print("Visualizing the bag")

    global leftButton_bag 
    global rightButton_bag 
    global block_bag 
    global leftGripper_bag 
    global rightGripper_bag 
    global jointState_bag



    # left_e0 = {}
    # left_e0['x'] = []
    # left_e0['y'] = []
    # left_e0['z'] = []

    # left_e1 = {}
    # left_e1['x'] = []
    # left_e1['y'] = []
    # left_e1['z'] = []

    # left_s0 = {}
    # left_s0['x'] = []
    # left_s0['y'] = []
    # left_s0['z'] = []

    # left_s1 = {}
    # left_s1['x'] = []
    # left_s1['y'] = []
    # left_s1['z'] = []

    # left_w0 = {}
    # left_w0['x'] = []
    # left_w0['y'] = []
    # left_w0['z'] = []
    
    # left_w1 = {}
    # left_w1['x'] = []
    # left_w1['y'] = []
    # left_w1['z'] = []
    
    # left_w2 = {}
    # left_w2['x'] = []
    # left_w2['y'] = []
    # left_w2['z'] = []

    # Indexes
    i_left_e0 = 3
    i_left_e1 = 4
    i_left_s0 = 5
    i_left_s1 = 6
    i_left_w0 = 7
    i_left_w1 = 8
    i_left_w2 = 9

    i_right_e0 = 12
    i_right_e1 = 13
    i_right_s0 = 14
    i_right_s1 = 15
    i_right_w0 = 16
    i_right_w1 = 17
    i_right_w2 = 18

    # Arrays of vals
    left_e0 = []
    left_e1 = []
    left_s0 = []
    left_s1 = []
    left_w0 = []
    left_w1 = []
    left_w2 = []

    right_e0 = []
    right_e1 = []
    right_s0 = []
    right_s1 = []
    right_w0 = []
    right_w1 = []
    right_w2 = []


    for topic, msg, t in jointState_bag.read_messages(topics=['robot/joint_states']):
        left_e0.append(msg.position[i_left_e0])
        left_e1.append(msg.position[i_left_e1])
        left_s0.append(msg.position[i_left_s0])
        left_s1.append(msg.position[i_left_s1])
        left_w0.append(msg.position[i_left_w0])
        left_w1.append(msg.position[i_left_w1])
        left_w2.append(msg.position[i_left_w2])

        right_e0.append(msg.position[i_right_e0])
        right_e1.append(msg.position[i_right_e1])
        right_s0.append(msg.position[i_right_s0])
        right_s1.append(msg.position[i_right_s1])
        right_w0.append(msg.position[i_right_w0])
        right_w1.append(msg.position[i_right_w1])
        right_w2.append(msg.position[i_right_w2])

        print("left_e0 " + msg.name[i_left_e0])
        print("right_e0 " + msg.name[i_right_e0])
    
    halfLen = len(left_e0)/2

    left_e0 = left_e0[:halfLen]
    left_e1 = left_e1[:halfLen]
    left_s0 = left_s0[:halfLen]
    left_s1 = left_s1[:halfLen]
    left_w0 = left_w0[:halfLen]
    left_w1 = left_w1[:halfLen]
    left_w2 = left_w2[:halfLen]

    right_e0 = right_e0[:halfLen]
    right_e1 = right_e1[:halfLen]
    right_s0 = right_s0[:halfLen]
    right_s1 = right_s1[:halfLen]
    right_w0 = right_w0[:halfLen]
    right_w1 = right_w1[:halfLen]
    right_w2 = right_w2[:halfLen]

    plt.plot(left_e0, 'r--', left_e1, 'r1',
             left_s0, 'b--', left_s1, 'b1', 
             left_w0, 'g--', left_w1, 'g1', left_w2, 'g^',
             right_e0, 'c--', right_e1, 'c1',
             right_s0, 'm--', right_s1, 'm1', 
             right_w0, 'y--', right_w1, 'y1', right_w2, 'y^')
    plt.show()


    """
    elevData_x_block = []
    elevData_y_block = []
    elevData_z_block = []
    c = 0
    for topic, msg, t in block_bag.read_messages(topics=['block_pose']):
        #if t < last_time:
        #    break
        #else:
        #    t = last_time
        elevData_x_block.append(msg.pose.position.x)
        elevData_y_block.append(msg.pose.position.y)
        elevData_z_block.append(msg.pose.position.z)
        # c+=1

    print("Added "+str(c)+" messages to plot")

    elevData_x_lbutton = []
    elevData_y_lbutton = []
    elevData_z_lbutton = []
    for topic, msg, t in leftButton_bag.read_messages(topics=['left_button_pose']):
        elevData_x_lbutton.append(msg.pose.position.x)
        elevData_y_lbutton.append(msg.pose.position.y)
        elevData_z_lbutton.append(msg.pose.position.z)

    elevData_x_rbutton = []
    elevData_y_rbutton = []
    elevData_z_rbutton = []
    for topic, msg, t in rightButton_bag.read_messages(topics=['right_button_pose']):
        elevData_x_rbutton.append(msg.pose.position.x)
        elevData_y_rbutton.append(msg.pose.position.y)
        elevData_z_rbutton.append(msg.pose.position.z)

    elevData_x_lgripper = []
    elevData_y_lgripper = []
    elevData_z_lgripper = []
    for topic, msg, t in leftGripper_bag.read_messages(topics=['left_gripper_pose']):
        elevData_x_lgripper.append(msg.x)
        elevData_y_lgripper.append(msg.y)
        elevData_z_lgripper.append(msg.z)

    elevData_x_rgripper = []
    elevData_y_rgripper = []
    elevData_z_rgripper = []
    for topic, msg, t in rightGripper_bag.read_messages(topics=['right_gripper_pose']):
        elevData_x_rgripper.append(msg.x)
        elevData_y_rgripper.append(msg.y)
        elevData_z_rgripper.append(msg.z)

    plt.plot(elevData_x_block, 'r--', elevData_y_block, 'r1', elevData_z_block, 'r^',
             elevData_x_lbutton, 'b--', elevData_y_lbutton, 'b1', elevData_z_lbutton, 'b^',
             elevData_x_rbutton, 'g--', elevData_y_rbutton, 'g1', elevData_z_rbutton, 'g^',
             elevData_x_lgripper, 'k--', elevData_y_lgripper, 'k1', elevData_z_lgripper, 'k^',
             elevData_x_rgripper, 'y--', elevData_y_rgripper, 'y1', elevData_z_rgripper, 'y^',)
    """


"""
header: 
  seq: 422998
  stamp: 
    secs: 8460
    nsecs: 369000000
  frame_id: ''
name: [head_pan, l_gripper_l_finger_joint, l_gripper_r_finger_joint, left_e0, left_e1, left_s0,
  left_s1, left_w0, left_w1, left_w2, r_gripper_l_finger_joint, r_gripper_r_finger_joint,
  right_e0, right_e1, right_s0, right_s1, right_w0, right_w1, right_w2]
position: [0.00037950190107327586, -7.320651061985309e-10, -9.007194895622202e-06, -1.1909722946993613, 1.9382078592691734, -0.07959292224814174, -0.9997521277253458, 0.6696652976179029, 1.0318677316390072, -0.4913284744172408, -3.0167292263687043e-09, -0.020833001482205276, 0.01544288531582172, 0.4941136969391611, -0.27231906855958954, 1.0470000130417105, 0.21297591992677134, 0.02276380988889848, 0.1320914918373033]
velocity: [-1.3409318717564828e-08, 1.5172544598044918e-07, 8.686871031266307e-07, 2.872689291693379e-07, -3.533709074745e-07, -2.2422995970956474e-07, -2.6765843977145595e-08, -7.418683112386929e-07, 4.974558504249287e-07, 1.2290406496279368e-06, -1.5094280632432012e-07, -1.5087212242602784e-07, -1.37584760108946e-06, -2.1939020649849428e-07, -8.306468466396253e-08, 3.676123786542591e-10, 2.6048955182016797e-05, -8.427936596269117e-07, 2.4698018362627136e-05]
effort: [0.0, 9.124296028588603e-07, 0.009008088544217203, -0.12429816348236145, -0.16163667914392832, 5.567546867979445e-06, -0.15937851568281758, 0.008033237475864041, -0.004958976126072656, 3.9155898079457074e-06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]




l_gripper_l_finger_joint
l_gripper_r_finger_joint

left_e0
left_e1
left_s0
left_s1
left_w0
left_w1
left_w2

r_gripper_l_finger_joint
r_gripper_r_finger_joint

right_e0
right_e1
right_s0
right_s1
right_w0
right_w1
right_w2

"""

def closeBags():
    global leftButton_bag 
    global rightButton_bag 
    global block_bag 
    global leftGripper_bag 
    global rightGripper_bag 

    leftButton_bag.close() 
    rightButton_bag.close() 
    block_bag.close() 
    leftGripper_bag.close() 
    rightGripper_bag.close() 
############### END: ROSbag handling


def execute_action(actionName, params):
    b = rospy.ServiceProxy(KB.getService(actionName), KB.getSrv(actionName))
    resp = None
    rospy.wait_for_service(KB.getService(actionName), timeout=60)
    try:
        if len(params) == 1:
            resp = b(params[0])
        elif len(params) == 2:
            resp = b(params[0], params[1])   
        elif len(params) == 3:
            resp = b(params[0], params[1], params[2])
        elif len(params) == 4:
            resp = b(params[0], params[1], params[2], params[3])
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
    # return(resp.success_bool)
    # return 1

def main():
    rospy.init_node("APV_node")
    rospy.wait_for_message("/robot/sim/started", Empty)

    rospy.Subscriber("left_button_pose", PoseStamped, handle_buttonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, handle_buttonRight)    
    rospy.Subscriber("block_pose", PoseStamped, handle_block)
    rospy.Subscriber("left_gripper_pose", Point, handle_gripperLeft)
    rospy.Subscriber("right_gripper_pose", Point, handle_gripperRight)

    rospy.Subscriber("robot/joint_states", JointState, handle_jointStates)

    s = rospy.Service("APV_srv", APVSrv, handle_APV)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
