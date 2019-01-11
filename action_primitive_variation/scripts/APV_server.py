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
from matplotlib.lines import Line2D

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
from util.bayesian_change_point import BayesianChangePoint
from util.general_vis import *
from util.ros_bag import RosBag
from action_primitive_variation.srv import *
from agent.srv import * 
from environment.msg import *

actionToVary = None 
gripper = None
obj = None
button = None

bags = []

block_bag = RosBag('block')
leftButton_bag = RosBag('leftButton')
rightButton_bag = RosBag('rightButton')
leftGripper_bag = RosBag('leftGripper')
rightGripper_bag = RosBag('rightGripper')
predicates_bag = RosBag('predicate')
jointState_bag = RosBag('jointState')

BlockPose = None
shouldRecord = False

# KB stuff not currently being used, but will be used for adding new actions
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

    openBags()
    shouldRecord = True
    execute_action(actionToVary, params)
    shouldRecord = False
    changePoints = extract_change_points()
    closeBags()

    return APVSrvResponse(changePoints)


############### START: Call back functions that check to see if ROSbag should be being recorded
def handle_jointStates(data):
    if shouldRecord is not False:
        try:
            jointState_bag.writeToBag('robot/joint_states', data)
        finally:
            pass

def handle_predicates(data):
    if shouldRecord is not False:
        try:
            predicates_bag.writeToBag('predicate_values', data)
        finally:
            pass

def handle_block(data):
    if shouldRecord is not False:
        try:
            block_bag.writeToBag('block_pose', data)
        finally:
            pass

def handle_buttonLeft(data):
    if shouldRecord is not False:
        try:
            leftButton_bag.writeToBag('left_button_pose', data)
        finally:
            pass

def handle_buttonRight(data):
    if shouldRecord is not False:
        try:
            rightButton_bag.writeToBag('right_button_pose', data)
        finally:
            pass

def handle_gripperLeft(data):
    if shouldRecord is not False:
        try:
            leftGripper_bag.writeToBag('left_gripper_pose', data)
            # jointState_bag.writeToBag('left_gripper_pose', data)
        finally:
            pass

def handle_gripperRight(data):
    if shouldRecord is not False:
        try:
            rightGripper_bag.writeToBag('right_gripper_pose', data)
        finally:
            pass
############### END: Call back functions that check to see if ROSbag should be being recorded

############### START: ROSbag handling

# def visualize_change_points():
#     bagData = jointState_bag.getVisualizableData()
#     segs = BayesianChangePoint(np.array(bagData), 'changePointData.csv')
#     ROSbag_with_CPs('jointState', bagData, segs) 

def extract_change_points():

    bagData =  leftGripper_bag.getVisualizableData()

    segs = BayesianChangePoint(np.array(bagData), 'changePointData.csv')
    cps = segs.getCompressedChangePoints()
    positionInfo = leftGripper_bag.getROSBagDataAtCps(segs.getCompressedChangePoints(), ['left_gripper_pose'], cps)
    ROSbag_with_CPs('leftGripper', bagData, segs)
    return positionInfo
    # return 1
    
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

"""

def closeBags():
    leftButton_bag.closeBag() 
    rightButton_bag.closeBag() 
    block_bag.closeBag() 
    leftGripper_bag.closeBag() 
    rightGripper_bag.closeBag() 
    jointState_bag.closeBag()

def openBags():
    leftButton_bag.openBag() 
    rightButton_bag.openBag() 
    block_bag.openBag() 
    leftGripper_bag.openBag() 
    rightGripper_bag.openBag() 
    jointState_bag.openBag()

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


def main():
    rospy.init_node("APV_node")
    rospy.wait_for_message("/robot/sim/started", Empty)

    global bags 
    # bags['block_bag'] = rosbag.Bag('block.bag' ,'w')
    # bags['leftButton_bag'] = rosbag.Bag('leftButton.bag' ,'w')
    # bags['rightButton_bag'] = rosbag.Bag('rightButton.bag' ,'w')
    # bags['leftGripper_bag'] = rosbag.Bag('leftGripper.bag' ,'w')
    # bags['rightGripper_bag'] = rosbag.Bag('rightGripper.bag' ,'w')
    # bags.append(RosBag('jointState'))


    rospy.Subscriber("left_button_pose", PoseStamped, handle_buttonLeft)
    rospy.Subscriber("right_button_pose", PoseStamped, handle_buttonRight)    
    rospy.Subscriber("block_pose", PoseStamped, handle_block)
    rospy.Subscriber("left_gripper_pose", PoseStamped, handle_gripperLeft)
    rospy.Subscriber("right_gripper_pose", PoseStamped, handle_gripperRight)
    rospy.Subscriber("robot/joint_states", JointState, handle_jointStates)
    rospy.Subscriber("predicate_values", PredicateList, handle_predicates)

    s = rospy.Service("APV_srv", APVSrv, handle_APV)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
