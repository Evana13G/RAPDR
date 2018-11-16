#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import rospy
import rospkg

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

from action_primitive_variation.srv import *
# from agent import KnowledgeBase












class KnowledgeBase(object):
    def __init__(self, actionPrimitiveNames, actionPrimitiveServices, actionPrimitiveSrvFiles):
        self.names = actionPrimitiveNames
        self.services = actionPrimitiveServices
        self.srvs = actionPrimitiveSrvFiles
        list_data = []
        dict_data = {}
        for i in range(len(self.names)):
            list_data_point = {}
            list_data_point['name'] = self.names[i]
            list_data_point['service'] = self.services[i]
            list_data_point['srv'] = self.srvs[i]
            list_data.append(list_data_point)
            dict_data[self.names[i]] = {}
            dict_data[self.names[i]]['service'] = self.services[i]
            dict_data[self.names[i]]['srv'] = self.srvs[i]    
        self.KB_list = list_data
        self.KB_dict = dict_data

    def getNames(self):
        return self.names

    def getServices(self):
        return self.services

    def getSrvs(self):
        return self.srvs

    def getService(self, actionName):
        return self.services[self.names.index(actionName)]

    def getSrv(self):
        return self.srvs[self.names.index(actionName)]

    def getAPData_list(self):
        return self.KB_list

    def getAPData_dict(self):
        return self.KB_dict

    def addAction(self, actionName, service, srv):
        self.names.append(actionName)
        self.services.append(service)
        self.srvs.append(srv)
        dict_data = {}
        list_data = {}
        list_data['name'] = actionName
        list_data['service'] = service
        list_data['srv'] = srv
        dict_data['service'] = service
        dict_data['srv'] = srv
        self.KB_list.append(list_data)
        self.KB_dict[actionName] = dict_data









actionToVary = None 
gripper = None
obj = None
button = None

AP_names = ['press_button', 'obtain_object']
AP_services = ['press_button_srv', 'obtain_object_srv']
AP_srvs = [PressButtonSrv, ObtainObjectSrv]
KB = KnowledgeBase(AP_names, AP_services, AP_srvs)


def handle_APV(req):
    print("(Received) Action to Vary: ")
    print(req.actionName)
    params = {}

    global actionToVary
    actionToVary = req.actionName
    global gripper
    gripper = req.gripper
    if gripper is not None: 
        params['gripper'] = gripper
    global obj
    obj = req.obj
    if obj is not None: 
        params['obj'] = obj
    global button
    button = req.button
    if button is not None: 
        params['button'] = button

    execute_action(actionToVary, params)


    return APVSrvResponse(1)

def execute_action(actionName, params):
    args = params.values()
    rospy.wait_for_service(KB.getService(actionName), timeout=60)
    try:
        b = rospy.ServiceProxy(KB.getService(actionName), KB.getSrv(actionName))

        resp = b(args)

        print(resp.success_bool)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def main():
    rospy.init_node("APV_node")
    # rospy.wait_for_message("/robot/sim/started", Empty)

    # rospy.Subscriber("left_button_pose", PoseStamped, getPoseButtonLeft)
    # rospy.Subscriber("right_button_pose", PoseStamped, getPoseButtonRight)
    # rospy.Subscriber("block_pose", PoseStamped, getPoseBlock)
    
    s = rospy.Service("APV_srv", APVSrv, handle_APV)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
