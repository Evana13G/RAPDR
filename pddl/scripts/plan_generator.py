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
from util.file_io import * 
from action_primitive_variation.srv import *
from agent.srv import * 
from pddl.srv import *


def generate_plan(req):

    domainFile = req.filename + '_domain.pddl'
    problemFile = req.filename + '_problem.pddl'
    solutionFile = req.filename + '.pddl.soln'
    
    dataFilepath = os.path.dirname(os.path.realpath(__file__)) + "/../data/"
    domainFilepath = dataFilepath + domainFile
    problemFilepath = dataFilepath + problemFile

    # write to the files
    writeToDomainFile(domainFilepath, 
                      req.domain.name, 
                      req.domain.requirements,
                      req.domain.types, 
                      req.domain.predicates, 
                      req.domain.actions)

    writeToProblemFile(problemFilepath, 
                       req.problem.task,
                       req.problem.domain,
                       req.problem.objects,
                       req.problem.init, 
                       req.problem.goals)

    pddlDriver = os.path.dirname(os.path.realpath(__file__)) + "/../../../pyperplan/src/pyperplan.py"

    # os.system('python3 ' + pddlDriver + ' ' + domainFilepath + ' ' + problemFilepath)



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

    return PlanGeneratorSrvResponse(1)


############### START: ROSbag handling

def main():
    rospy.init_node("plan_generator_node")
    rospy.wait_for_message("/robot/sim/started", Empty)

    s = rospy.Service("plan_generator_srv", PlanGeneratorSrv, generate_plan)


    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
