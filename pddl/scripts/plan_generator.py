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


# AP_names = ['press_button', 'obtain_object']

# AP_services = ['press_button_srv', 'obtain_object_srv']
# AP_srvs = [PressButtonSrv, ObtainObjectSrv]

# KB = KnowledgeBase(AP_names, AP_services, AP_srvs)

def generate_plan(req):

    # load file and parse 
    # solution_file = req.full_plan_filepath
    # plan = []

    # with open(solution_file) as f:
    #     plan_data = f.readlines()

    # for full_action in plan_data:
    #     data = full_action.replace(")\n", "").replace("(", "")

    #     args = data.split()
    #     action = {}
    #     params = []
    #     action['actionName'] = args[0]
    #     params.append(args[1])
    #     params.append(args[3])
    #     action['params'] = params
    #     plan.append(action)

    return PlanGeneratorSrvResponse("yo")


############### START: ROSbag handling

def main():
    rospy.init_node("plan_generator_node")
    rospy.wait_for_message("/robot/sim/started", Empty)

    s = rospy.Service("plan_generator_srv", PlanGeneratorSrv, generate_plan)


    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
