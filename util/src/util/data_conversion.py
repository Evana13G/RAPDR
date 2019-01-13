
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
# import baxter_interface
# from environment.srv import *
# from environment.msg import *
# from util.image_converter import ImageConverter

def is_touching(object1_loc, object2_loc, epsilon=0.135):
    if ((object1_loc is not None) and (object2_loc is not None)):
        obj1 = np.array((object1_loc.pose.position.x, 
                            object1_loc.pose.position.y,
                            object1_loc.pose.position.z)) 
        obj2 = np.array((object2_loc.pose.position.x, 
                            object2_loc.pose.position.y,
                            object2_loc.pose.position.z)) 
        if np.linalg.norm(obj1 - obj2) < epsilon:
            return True 
        return False


def pddlStringFormat(predicates_list):
    stringList = []
    for pred in predicates_list:
        if pred.operator == "at":
            stringList.append(str(pred.operator) + '(' + str(pred.object) + ', (' + 
                              str(round(pred.locationInformation.pose.position.x, 2)) + ', ' + 
                              str(round(pred.locationInformation.pose.position.y, 2)) + ', ' + 
                              str(round(pred.locationInformation.pose.position.z, 2)) + '))')
        else:
            stringList.append(str(pred.operator) + '(' + str(pred.object) + ')')
    return stringList

def pddlObjectsStringFormat(predicates_list):
    waypoints = []
    buttons = ''
    grippers = ''
    objs = ''
    for pred in predicates_list:
        if pred.operator == "at":
            loc = (str(round(pred.locationInformation.pose.position.x, 2)) + ',' + 
                   str(round(pred.locationInformation.pose.position.y, 2)) + ',' + 
                   str(round(pred.locationInformation.pose.position.z, 2))) + ' '
            waypoints.append(loc)
        if 'button' in str(pred.object):
            buttons = buttons + str(pred.object) + ' '
        elif 'gripper' in str(pred.object):
            grippers = grippers + str(pred.object) + ' '
        else:
            objs = objs + str(pred.object) + ' '

    waypoints = list(set(waypoints))
    waypoints = ''.join(waypoints) + '- waypoint'
    buttons = buttons + '- button'
    grippers = grippers + '- gripper'
    objs = objs + '- obj'
    return [waypoints, buttons, grippers, objs]

def pddlObjects(predicates_list):
    waypoints = []
    buttons = []
    grippers = []
    objs = []
    for pred in predicates_list.predicates:
        if pred.operator == "at":
            loc = (str(round(pred.locationInformation.pose.position.x, 2)) + ',' + 
                   str(round(pred.locationInformation.pose.position.y, 2)) + ',' + 
                   str(round(pred.locationInformation.pose.position.z, 2))) + ' '
            waypoints.append(loc)
        if 'button' in str(pred.object):
            buttons.append(str(pred.object))
        elif 'gripper' in str(pred.object):
            modifiedName = str(pred.object).replace('_gripper', '')
            grippers.append(modifiedName)
        else:
            objs.append(str(pred.object))

    waypoints = list(set(waypoints))
    buttons = list(set(buttons))
    grippers = list(set(grippers))
    objs = list(set(objs))

    objects = {}
    objects['types'] = ['waypoints', 'buttons', 'grippers', 'objs']
    objects['waypoint'] = waypoints
    objects['button'] = buttons
    objects['gripper'] = grippers
    objects['obj'] = objs

    return objects

def pddlInitStringFormat(predicates_list):
    stringList = []
    for pred in predicates_list:
        if pred.operator == "at":
            loc = (str(round(pred.locationInformation.pose.position.x, 2)) + ',' + 
                   str(round(pred.locationInformation.pose.position.y, 2)) + ',' + 
                   str(round(pred.locationInformation.pose.position.z, 2))) 
            if 'button' in str(pred.object):
                stringList.append('(button_at ' + pred.object + ' ' + loc + ')')
            elif 'gripper' in str(pred.object):
                stringList.append('(gripper_at ' + pred.object + ' ' + loc + ')')
            else:
                stringList.append('(object_at ' + pred.object + ' ' + loc + ')')
        else:
            stringList.append('(' + pred.operator + ' ' + pred.object + ')')
    return stringList

    # (gripper_at left loc0a)
    # (gripper_at right loc0b)
    # (button_at left_button loc1)
    # (button_at right_button loc2)
    # (object_at block loc3)