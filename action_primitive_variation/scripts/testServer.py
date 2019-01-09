#!/usr/bin/env python

from agent.srv import PressButtonSrv, CloseGripperSrv, OpenGripperSrv, ObtainObjectSrv
import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from action_primitive_variation.srv import APVSrv
from util.bayesian_change_point import BayesianChangePoint
import numpy as np

# import cpdetect 
# import sys, argparse, csv
import matplotlib.pyplot as plt # if no vis, delete  
import pandas as pd
from sklearn.cluster import AgglomerativeClustering

import scipy.cluster.hierarchy as shc

# def compressChangePoints(data):
#     content = []
#     i = 0
#     # for traj in data:
#     np_traj = np.array(getBooleanTraj(data))

#     cluster = AgglomerativeClustering(n_clusters=3, affinity='euclidean', linkage='ward')  
#     cluster.fit_predict(np_traj)
#     # print("Clusters for " + str(i) + "(th/st/nd/rd) trajectory: ") 
#     # print(cluster.labels_)
#     # content.append(cluster)
#     # i = i + 1
#     return cluster

# def getBooleanTraj(traj):
#     boolArray = []
#     for i in range(2000):
#         if i in traj:
#             boolArray.append([i, i])
#         # else:
#         #     boolArray.append([i, 0])
#     return boolArray


def main():
    rospy.init_node("APV_test_node")
    rospy.wait_for_service('APV_srv', timeout=60)
    try:
        b = rospy.ServiceProxy('APV_srv', APVSrv)
        # resp = b('obtain_object', 'left', 'block', None)
        resp = b('press_button', 'left', 'left_button', None)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

    # data = np.array([104.0, 27.0, 1274.0, 172.0, 1109.0, 392.0, 1084.0, 504.0, 1247.0, 1476.0, 1359.0])
    # np_traj = np.array(getBooleanTraj(data))
    # thresh = 300
    # clusters = shc.fclusterdata(np_traj, thresh, criterion="distance")

    # # plotting
    # plt.scatter(*np.transpose(np_traj), c=clusters)
    # plt.axis("equal")
    # title = "threshold: %f, number of clusters: %d" % (thresh, len(set(clusters)))
    # plt.title(title)
    # plt.show()

    # plt.figure(figsize=(10, 7))  
    # plt.title("Bayesian Change Points")  
    # linkage_matrix = shc.linkage(np_traj, method='ward')
    # # print(shc.linkage(np_traj, method='centroid'))
    # dend = shc.dendrogram(linkage_matrix)
    # cluster = compressChangePoints(data)
    # plt.scatter(np_traj[:,0],np_traj[:,1], c=cluster.labels_, cmap='rainbow')  
    # plt.show()


if __name__ == "__main__":
    main()
