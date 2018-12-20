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

import cpdetect 
import numpy as np
import sys, argparse, csv

class BayesianChangePoint(object):
    def __init__(self, _data):
        self.data = _data

    def getData(self):
        return self.data

    def detectChangePoints(self):
        detector = cpdetect.cpDetector(self.data, distribution='normal', log_odds_threshold=0)
        detector.detect_cp()
        detector.to_csv('changePointData.csv')

    def pullOutChangePoints(self):
        content = []
        with open('changePointData.csv', 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                content.append(int(row[2]))
        return content
