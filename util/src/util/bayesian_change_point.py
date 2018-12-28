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
from sklearn.svm import LinearSVC
from sklearn.datasets import make_classification
import cpdetect 
import numpy as np
import sys, argparse, csv

class BayesianChangePoint(object):
    def __init__(self, _data, _filename):
        self.rawData = _data
        self.filename = _filename
        cp_set, cp_list = self.detectChangePoints()
        self.trajectoryChangePoints = cp_set
        self.fullListChangePoints = cp_list
        # self.compressedChangePoints = self.compressChangePoints()
        self.compressChangePoints()

    def detectChangePoints(self):
        detector = cpdetect.cpDetector(self.rawData, distribution='normal', log_odds_threshold=0)
        detector.detect_cp()
        detector.to_csv(self.filename)
        return self.csvExtractCps()

    def csvExtractCps(self):
        content = {}
        content_list = []
        trajs = ['traj_0', 'traj_1', 'traj_2', 'traj_3', 'traj_4', 'traj_5', 'traj_6']
        for traj in trajs:
            content[traj] = []
        print(content)
        with open(self.filename, 'rb') as csvfile:
            reader = csv.reader(self.filename, delimiter=',')
            next(reader)
            for row in reader:
                print(row)
                # content[row[0]].append(int(row[2]))
                content_list.append(int(row[2]))
        return content, content_list

    def compressChangePoints(self):
        content = {}
        for traj in self.trajectoryChangePoints:
            X, y = make_classification(n_features=4, random_state=0)
            clf = LinearSVC(random_state=0, tol=1e-5)
            clf.fit(X, y)
            print(clf.coef_)
            print(clf.intercept_)
            print(clf.predict([[0, 0, 0, 0]]))


    def getChangePoints(self, trajectory):
        return self.trajectoryChangePoints[trajectory]