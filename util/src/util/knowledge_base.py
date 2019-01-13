#!/usr/bin/env python

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
)

from std_msgs.msg import (
    Header,
    Empty,
)

from action_primitive_variation.srv import *
from agent.srv import *

class KnowledgeBase(object):
    def __init__(self):
        _domain = 'rapdr'
        _reqs = ['strips', 'typing', 'fluents', 'disjunctive-preconditions']
        _types = []
        _preds = []
        _actions = []

        _types.append(Type('object', ['location', 'obj', 'gripper', 'button']))
        _types.append(Type('location', ['waypoint']))

        _preds.append(Predicate('gripper_at', [Variable('?g', 'gripper'), Variable('?wp', 'waypoint')]))
        _preds.append(Predicate('object_at', [Variable('?o', 'obj'), Variable('?wp', 'waypoint')]))
        _preds.append(Predicate('button_at', [Variable('?b', 'button'), Variable('?wp', 'waypoint')]))
        _preds.append(Predicate('pressed', [Variable('?b', 'button')]))
        _preds.append(Predicate('is_visible', [Variable('?o', 'obj')]))

        
        _a1 = Action('obtain_object', [], [], [], ObtainObjectSrv)
        _a1.addVar(Variable('?g', 'gripper'))
        _a1.addVar(Variable('?loc0', 'waypoint'))
        _a1.addVar(Variable('?o', 'obj'))
        _a1.addVar(Variable('?loc1', 'waypoint'))
        _a1.addPreCond(StaticPredicate('gripper_at', ['?g', '?loc0']))
        _a1.addPreCond(StaticPredicate('object_at', ['?o', '?loc1']))
        _a1.addEffect(StaticPredicate('gripper_at', ['?g', '?loc0']))
        _a1.addEffect(StaticPredicate('not', [StaticPredicate('object_at', ['?o', '?loc1'])]))

        _a2 = Action('press_button', [], [], [], PressButtonSrv)
        _a2.addVar(Variable('?g', 'gripper'))
        _a2.addVar(Variable('?loc0', 'waypoint'))
        _a2.addVar(Variable('?b', 'button'))
        _a2.addVar(Variable('?loc1', 'waypoint'))
        _a2.addPreCond(StaticPredicate('gripper_at', ['?g', '?loc0']))
        _a2.addPreCond(StaticPredicate('button_at', ['?b', '?loc1']))
        _a2.addEffect(StaticPredicate('gripper_at', ['?g', '?loc0']))
        _a2.addEffect(StaticPredicate('pressed', ['?b']))

        _actions.append(_a1)
        _actions.append(_a2)

        # _actions.append(Action('obtain_object', [Variable('?g', 'gripper'), Variable('?loc0', 'waypoint'), Variable('?o', 'object'), Variable('?loc1', 'waypoint')], 
        #                                         [StaticPredicate('gripper_at', ['?g', '?loc0']), StaticPredicate('object_at', ['?o', '?loc1'])],
        #                                         [StaticPredicate('gripper_at', ['?g', '?loc0']), StaticPredicate('not', [StaticPredicate('object_at', ['?o', '?loc1'])])]))
        # _actions.append(Action('press_button',  [Variable('?g', 'gripper'), Variable('?loc0', 'waypoint'), Variable('?b', 'button'), Variable('?loc1', 'waypoint')], 
        #                                         [StaticPredicate('gripper_at', ['?g', '?loc0']), StaticPredicate('button_at', ['?b', '?loc1'])],
        #                                         [StaticPredicate('gripper_at', ['?g', '?loc0']), StaticPredicate('pressed', ['?b'])]))

        self.domain = _domain
        self.requirements = _reqs
        self.types = _types
        self.predicates = _preds
        self.actions = _actions


    def getDomainData(self):
        
        data = {}
        _reqs = []
        _types = []
        _preds = []
        _acts = []
        
        for r in self.requirements:
            _reqs.append(':' + r)
        for t in self.types:
            _types.append(str(t))
        for p in self.predicates:
            _preds.append(str(p))
        for a in self.actions:
            _acts.append(str(a))
            
        data['domain'] = self.domain
        data['requirements'] = _reqs
        data['types'] = _types
        data['predicates'] = _preds
        data['actions'] = _acts

        return data

    def getService(self, actionName):
        for action in self.actions:
            if action.getName() == actionName:
                return action.getSrv()

    def getServiceFile(self, actionName):
        for action in self.actions:
            if action.getName() == actionName:
                return action.getSrvFile()

    def getActions(self):
        return self.actions

class Type(object):
    def __init__(self, parent, children):
        self.parentType = parent
        self.childrenTypes = children

    def __str__(self):
        s = ''
        for t in self.childrenTypes:
            s = s + t + ' '

        s = s + '- ' + self.parentType
        return s

# class Predicate(object):
class Predicate(object):
    def __init__(self, _operator, _vars):
        self.operator = _operator
        self.vars = _vars

    def __str__(self):
        args = ''
        for v in self.vars:
            args = args + ' ' + str(v)
        return "(" + self.operator + args + ")"


class StaticPredicate():
    def __init__(self, _operator, _args):
        self.operator = _operator
        self.vars = _args

    def __str__(self):
        args = ''
        for var in self.vars:
            args = args + str(var) + ' '
        return "(" + self.operator + ' ' + args + ")"


class Variable():
    def __init__(self, _var, _type):
        self.variable = _var
        self.type = _type

    def getName(self):
        return self.variable

    def getType(self):
        return self.type

    def __str__(self):
        return self.variable + ' - ' + self.type


class Action():
    def __init__(self, actionName, _params, _preConds, _effects, _srvFile):
        self.name = actionName
        self.params = _params
        self.preconditions = _preConds # list of predicates (recursive)
        self.effects = _effects
        self.srv = actionName + '_srv'
        self.srvFile = _srvFile

    def addVar(self, var):
        self.params.append(var) # check to make sure this actually sets it 

    def addPreCond(self, predicate):
        self.preconditions.append(predicate)

    def addEffect(self, predicate):
        self.effects.append(predicate)

    def getName(self):
        return self.name 

    def getSrv(self):
        return self.srv

    def getSrvFile(self):
        return self.srvFile

    def getNonLocationVars(self):
        args = []
        for v in self.params:
            t = v.getType()
            if (t != 'waypoint') and (t != 'location'):
                args.append(t)
        return args

    def __str__(self):
        s = '(:action ' + self.name + '\n'
        s = s + '    :parameters ('
        for p in self.params:
            s = s + str(p) + ' '
        s = s + ')\n'

        if len(self.preconditions) > 1:    
            s = s + '    :precondition (and'
            for pcs in self.preconditions:
                s = s + '\n        ' + str(pcs)
            s = s + ')\n'
        else:
            s = s + '    :precondition ' + str(self.preconditions[0]) + '\n'

        if len(self.effects) > 1:  
            s = s + '    :effect (and'
            for e in self.effects:
                s = s + '\n        ' + str(e)
            s = s + ')\n)'
        else:
            s = s + ':effect ' + str(self.effects[0]) + '\n)'
        return s
