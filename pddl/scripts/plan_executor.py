#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np
import os 
import copy

import rospy
import rospkg
import rosbag

from std_msgs.msg import (
    Header,
    Empty,
)

from tf.transformations import *

from util.knowledge_base import KnowledgeBase
from action_primitive_variation.srv import *
from agent.srv import * 
from pddl.srv import *

actionToVary = None 
gripper = None
obj = None
button = None

AP_names = ['press_button', 'obtain_object']

AP_services = ['press_button_srv', 'obtain_object_srv']
AP_srvs = [PressButtonSrv, ObtainObjectSrv]

KB = KnowledgeBase(AP_names, AP_services, AP_srvs)

def handle_plan(req):

    # load file and parse 
    solution_file = req.full_plan_filepath
    plan = []

    with open(solution_file) as f:
        plan_data = f.readlines()

    for full_action in plan_data:
        data = full_action.replace(")\n", "").replace("(", "")

        args = data.split()
        action = {}
        params = []
        action['actionName'] = args[0]
        params.append(args[1])
        params.append(args[3])
        action['params'] = params
        plan.append(action)

    for action in plan:
        execute_action(action['actionName'], action['params'])

    return PlanExecutorSrvResponse(1)


############### START: ROSbag handling

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
    rospy.init_node("plan_executor_node")
    rospy.wait_for_message("/robot/sim/started", Empty)

    # rospy.Subscriber("left_button_pose", PoseStamped, handle_buttonLeft)

    s = rospy.Service("plan_executor_srv", PlanExecutorSrv, handle_plan)
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
