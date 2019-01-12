
#!/usr/bin/env python

import sys
import cv2
import math
import time

import argparse
import struct
import sys
import copy
import numpy as np
import rospy
import rospkg


from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    PoseArray,
    PoseWithCovarianceStamped,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
    Empty,
)

# from baxter_core_msgs.srv import (
#     SolvePositionIK,
#     SolvePositionIKRequest,
# )

from tf.transformations import *



# domain = Domain(domainName, requirements, types, predicates, actions)


def writeToDomainFile(filePath, _name, _reqs, _types, _preds, _actions):

    define = 'define (domain ' + _name + ')\n\n'

    # maybe use join instead 
    reqs = '(:requirements'
    for r in _reqs:
        reqs = reqs + ' ' + r
    reqs = reqs + ')\n\n'

    types = '(:types\n' 
    for t in _types:
        types = types + '    ' + t + '\n'
    types = types + ')\n\n'

    preds = '(:predicates\n'
    for p in _preds:
        preds = preds + '    ' + p + '\n'
    preds = preds + ')\n\n'

    actions = ''
    for a in _actions:
        actions = actions + a + '\n\n'

    # open file 
    with open(filePath, 'w') as f:
        f.write('(')
        f.write(define)
        f.write(reqs)
        f.write(types)
        f.write(preds)
        f.write(actions)
        f.write(')')



def writeToProblemFile(filePath, _task, _domain, _objs, _init, _goals):

    define = 'define (problem  ' + _task + ')\n\n'
    domain = '(:domain ' + _domain + ')\n\n'

    # # maybe use join instead 
    objs = '(:objects'
    for o in _objs:
        objs = objs + '\n    ' + o
    objs = objs + '\n)\n\n'

    init = '(:init' 
    for i in _init:
        init = init + '\n    ' + i
    init = init + '\n)\n\n'

    goals = '(:goal (and '
    for g in _goals:
        goals = goals + '\n    ' + g
    goals = goals + ')\n)\n\n'


    # # open file 
    with open(filePath, 'w') as f:
        f.write('(')
        f.write(define)
        f.write(domain)
        f.write(objs)
        f.write(init)
        f.write(goals)
        f.write(')')


