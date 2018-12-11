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

    def getSrv(self, actionName):
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