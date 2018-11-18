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

class KnowledgeBase(object):
    def __init__(self, actionPrimitiveNames, actionPrimitiveServers, actionPrimitiveSrvFiles):
        self.names = actionPrimitiveNames
        self.servers = actionPrimitiveServers
        self.srvs = actionPrimitiveSrvFiles
        list_data = {}
        for i in range(len(self.names)):
            list_data[self.names[i]] = {}
            list_data[self.names[i]]['server'] = self.servers[i]
            list_data[self.names[i]]['srv'] = self.srvs[i]    
        self.KB_list = list_data

        self.KB_dict = 

    def getNames(self):
        return self.names

    def getServers(self):
        return self.servers

    def getSrvs(self):
        return self.srvs

    def getAPData():

        return 