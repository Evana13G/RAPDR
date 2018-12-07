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
bag = rosbag.Bag('APV.bag' ,'w')
BlockPose = None
shouldRecord = False

AP_names = ['press_button', 'obtain_object']
AP_services = ['press_button_srv', 'obtain_object_srv']
AP_srvs = [PressButtonSrv, ObtainObjectSrv]
KB = KnowledgeBase(AP_names, AP_services, AP_srvs)

def handle_APV(req):
    params = []

    global actionToVary
    actionToVary = req.actionName
    global gripper
    gripper = req.gripper
    if gripper is not '': 
        params.append(gripper)
    global obj
    obj = req.obj
    if obj is not '': 
        params.append(obj)
    global button
    button = req.button
    if button is not '': 
        params.append(button)

    global shouldRecord

    shouldRecord = True
    execute_action(actionToVary, params)
    shouldRecord = False
    global bag 

    elevData = []
    for topic, msg, t in bag.read_messages(topics=['block_pose']):
        elevData.append(msg.pose.position.z)
    bag.close()

    plt.plot(elevData)
    plt.show()
    
    return APVSrvResponse(1)

def setPoseBlock(data):
    global BlockPose
    BlockPose = data
    global shouldRecord
    if shouldRecord is not False:
        write_to_ROSbag()

def getPoseBlock():
    global BlockPose
    return copy.deepcopy(BlockPose)

def write_to_ROSbag():
    global bag
    try:
        # data = str(getPoseBlock().pose.position.x)
        bag.write('block_pose', getPoseBlock())
    finally:
        pass
        # bag.close()


#     header = { 'topic' : topic, 'type' : msg.__class__._type, 'md5sum' : msg.__class__._md5sum, 'message_definition' : msg._full_text }
# AttributeError: type object 'str' has no attribute '_type'




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
    # rospy.wait_for_message("/robot/sim/started", Empty)
    # rospy.Subscriber("left_button_pose", PoseStamped, getPoseButtonLeft)
    # rospy.Subscriber("right_button_pose", PoseStamped, getPoseButtonRight)
    rospy.Subscriber("block_pose", PoseStamped, setPoseBlock)
    s = rospy.Service("APV_srv", APVSrv, handle_APV)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
