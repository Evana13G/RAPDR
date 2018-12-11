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

leftButton_bag = rosbag.Bag('leftButton.bag' ,'w')
rightButton_bag = rosbag.Bag('rightButton.bag' ,'w')
block_bag = rosbag.Bag('block.bag' ,'w')
leftGripper_bag = rosbag.Bag('leftGripper.bag' ,'w')
rightGripper_bag = rosbag.Bag('rightGripper.bag' ,'w')

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
    shouldRecord = True
    execute_action(actionToVary, params)
    shouldRecord = False
    # closeBags()
    visualize_ROSbag_data()
    
    return APVSrvResponse(1)


############### START: Call back functions that check to see if ROSbag should be being recorded
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
            block_bag.write('left_button_pose', data)
        finally:
            pass

def handle_buttonRight(data):
    global shouldRecord
    if shouldRecord is not False:
        try:
            block_bag.write('right_button_pose', data)
        finally:
            pass

def handle_gripperLeft(data):
    global shouldRecord
    if shouldRecord is not False:
        try:
            block_bag.write('left_gripper_pose', data)
        finally:
            pass

def handle_gripperRight(data):
    global shouldRecord
    if shouldRecord is not False:
        try:
            block_bag.write('right_gripper_pose', data)
        finally:
            pass


############### END: Call back functions that check to see if ROSbag should be being recorded

############### START: ROSbag handling
def visualize_ROSbag_data():

    global leftButton_bag 
    global rightButton_bag 
    global block_bag 
    global leftGripper_bag 
    global rightGripper_bag 

    elevData_x = []
    elevData_y = []
    elevData_z = []

    for topic, msg, t in block_bag.read_messages(topics=['block_pose']):
        elevData_x.append(msg.pose.position.x)
        elevData_y.append(msg.pose.position.y)
        elevData_z.append(msg.pose.position.z)
    block_bag.close()


    # figure
    # plt.plot(elevData_x)
    # plt.show()
    # figure
    # plt.plot(elevData_y)
    # plt.show()
    # figure
    # plt.plot(elevData_z)
    # plt.show()



#     header = { 'topic' : topic, 'type' : msg.__class__._type, 'md5sum' : msg.__class__._md5sum, 'message_definition' : msg._full_text }
# AttributeError: type object 'str' has no attribute '_type'

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

    s = rospy.Service("APV_srv", APVSrv, handle_APV)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
