
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

from tf.transformations import *
from kb_subclasses import *


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
            stringList.append(str(pred.operator) + '(' + str(pred.object) + ', (' + poseStampedToString(pred.locationInformation) + '))')
        else:
            stringList.append(str(pred.operator) + '(' + str(pred.object) + ')')
    return stringList

def pddlObjectsStringFormat(predicates_list):
    strData = []
    objData = pddlObjects(predicates_list, False)
    for objType in ['waypoint', 'button', 'gripper', 'obj']:
        s = ''
        for item in objData[objType]:
            s = s + item + ' '
        s = s + '- ' + objType
        strData.append(s)
    return strData


def pddlObjects(predicates_list, mod=True):
    waypoints = []
    buttons = []
    grippers = []
    objs = []
    for pred in predicates_list:
        if pred.operator == "at":
            if mod == True:
                loc = poseStampedToString(pred.locationInformation) + ' '
                waypoints.append(loc)
            else: 
                loc = poseStampedToString(pred.locationInformation)
                waypoints.append(loc)
        if 'button' in str(pred.object):
            buttons.append(str(pred.object))
        elif 'gripper' in str(pred.object):
            grippers.append(str(pred.object))
        else:
            objs.append(str(pred.object))

    waypoints = list(set(waypoints))
    buttons = list(set(buttons))
    grippers = list(set(grippers))
    objs = list(set(objs))

    objects = {}
    objects['types'] = ['waypoint', 'button', 'gripper', 'obj']
    objects['waypoint'] = waypoints
    objects['button'] = buttons
    objects['gripper'] = grippers
    objects['obj'] = objs

    return objects

def pddlInitStringFormat(predicates_list):
    stringList = []
    for pred in predicates_list:
        if pred.operator == "at":
            loc = poseStampedToString(pred.locationInformation)
            if 'button' in str(pred.object):
                stringList.append('(button_at ' + pred.object + ' ' + loc + ')')
            elif 'gripper' in str(pred.object):
                stringList.append('(gripper_at ' + pred.object + ' ' + loc + ')')
            else:
                stringList.append('(obj_at ' + pred.object + ' ' + loc + ')')
        else:
            stringList.append('(' + pred.operator + ' ' + pred.object + ')')
    return stringList

    # (gripper_at left loc0a)
    # (gripper_at right loc0b)
    # (button_at left_button loc1)
    # (button_at right_button loc2)
    # (object_at block loc3)

def pddlCondsKBFormat(_vars, args, predicates_list):
    predList = []
    varSymbols = []
    coorespondingVarTypes = []
    for v in _vars:
        varSymbols.append(v.getName())
        coorespondingVarTypes.append(v.getType())

    print(" **** Info on pddlKBFormat **** ")
    print(" **** varSymbols **** ")
    print(varSymbols)
    print(" **** coorespondingVarTypes **** ")
    print(coorespondingVarTypes)
    print(" **** args **** ")
    print(args)

    for pred in predicates_list.predicates:
        if (pred.object in args):
            print('object = ' + pred.object)

            i_obj = args.index(pred.object)
            print('i_obj = ' + str(i_obj))

            _symbol_obj = varSymbols[i_obj]
            print('_symbol_obj = ' + str(_symbol_obj))

            if pred.operator == "at":

                i_loc = args.index(poseStampedToString(pred.locationInformation))
                print('i_loc = ' + str(i_loc))

                _symbol_loc = varSymbols[i_loc]
                print('_symbol_loc = ' + str(_symbol_loc))

                _type_obj = coorespondingVarTypes[i_obj]
                print('_type_obj = ' + str(_type_obj))


                predList.append(StaticPredicate(_type_obj + '_at ', [_symbol_obj, _symbol_loc]))
            else:
                predList.append(StaticPredicate(pred.operator, [_symbol_obj]))

    return predList

def getPredicateDiffs(predList1, predList2):
    diffs = []
    p1 = pddlInitStringFormat(predList1.predicates)
    p2 = pddlInitStringFormat(predList2.predicates)
    for i in range(len(p1)):
        if p1[i] not in p2:
            diffs.append(predList1.predicates[i])
    for i in range(len(p2)):
        if p2[i] not in p1:
            diffs.append(predList2.predicates[i])
    return list(set(diffs))

def poseStampedToString(val):
    x = round(val.pose.position.x, 1)
    y = round(val.pose.position.y, 1)
    z = round(val.pose.position.z, 1)
    if x == -0.0:
        x = 0.0
    if y == -0.0:
        y = 0.0
    if z == -0.0:
        z = 0.0
    return (str(x) + ',' + 
            str(y) + ',' + 
            str(z))

def getElementDiffs(predList1, predList2, OGargs=[]):
    nonRepeatingDiffs =[]
    diffs = getPredicateDiffs(predList1, predList2)
    print('**** diffs in getElemDiffs')
    print(diffs)
    print('***** ogargs')
    print(OGargs)
    for o in diffs:
        if o not in OGargs:
            nonRepeatingDiffs.append(o.object)
    return list(set(nonRepeatingDiffs))

def typeChecker(elementName, types=["obj", "gripper", "button", "waypoint"]):
    for t in types:
        if t in str(elementName):
            return t
        return "obj"

def getBoundLocs(preds):
    locVars = []
    coorespondingArgs = []
    for pred in preds:
        if pred.operator  == 'at':
            coorespondingArgs.append(poseStampedToString(pred.locationInformation))

    coorespondingArgs = list(set(coorespondingArgs))

    for i in range(len(coorespondingArgs)):
        locVars.append(Variable('?loc'+str(i), 'waypoint'))

    return locVars, coorespondingArgs 

def removeNoneInstances(lst):
    newLst = []
    for elem in lst:
        if elem is not None:
            newLst.append(elem)
    return newLst

def pddlActionKBFormat(_vars, args, preCondsPredList, effectsPredList):
    diffsObjs = []
    diffsVars = []
    templatedVars = []
    args = removeNoneInstances(args)
    print(" **** Passed Args **** ")
    for a in args:
        print(a)
    for _v in copy.deepcopy(_vars):
        if _v.getType() != 'waypoint':
            templatedVars.append(_v)

    diffs = getElementDiffs(preCondsPredList, effectsPredList, args) # consider these in appending more templated vars  
    for i in range(len(diffs)):
        templatedVars.append(Variable('?'+str(i), typeChecker(diffs[i])))

    locVars, locArgs = getBoundLocs(preCondsPredList.predicates + effectsPredList.predicates)
    templatedVars = templatedVars + locVars
    args = args + diffs + locArgs # merge the arguments

    #print("Check correspondence of vars: ")
    #print(" **** Templated Vars **** ")
    #for a in templatedVars:
    #    print(a.getName() + ' - ' + a.getType())

    preconds = pddlCondsKBFormat(templatedVars, args, preCondsPredList)
    effects = pddlCondsKBFormat(templatedVars, args, effectsPredList)
    # This is tricky, because I am not sure if the order is retained. I think it is though

    return templatedVars, preconds, effects
