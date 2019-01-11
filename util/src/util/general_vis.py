#!/usr/bin/env python

# import numpy as np

# import rospy
# import rospkg
import rosbag
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


# from tf.transformations import *
# import baxter_interface
# from util.knowledge_base import KnowledgeBase
from util.bayesian_change_point import BayesianChangePoint
from util.ros_bag import RosBag

def ROSbag_with_CPs(bagName, bagData, cpData):


    if bagName == 'jointState':
        # visData = jointState_bag.getVisualizableData()
        # n = jointState_bag.getNumberOfTrajectories()
        # cpData.detectChangePoints()

        cps_0 = cpData.getTrajectoryBasedPoints('traj_0')
        cps_1 = cpData.getTrajectoryBasedPoints('traj_1')
        cps_2 = cpData.getTrajectoryBasedPoints('traj_2')
        cps_3 = cpData.getTrajectoryBasedPoints('traj_3')
        cps_4 = cpData.getTrajectoryBasedPoints('traj_4')
        cps_5 = cpData.getTrajectoryBasedPoints('traj_5')
        cps_6 = cpData.getTrajectoryBasedPoints('traj_6')

        plt.plot(bagData[0], 'r--', label='left_e0')
        plt.plot(bagData[1], 'r1', label='left_e1')
        plt.plot(bagData[2], 'b--', label='left_s0')
        plt.plot(bagData[3], 'b1', label='left_s1')
        plt.plot(bagData[4], 'g--', label='left_w0')
        plt.plot(bagData[5], 'g1', label='left_w1')
        plt.plot(bagData[6], 'g^', label='left_w2')



            # red
            # blue
            # green

        colrs = ['m', 'c', 'y', '#a3c2c2', '#ff9933', '#aa80ff']        
        compressedCps = cpData.getGroupedData()

        # colrs = ['m', 'c', 'y', 'k', '#ff9933', '#aa80ff', '#a3c2c2']
        for group_number in range(len(compressedCps) - 1):
            clr = colrs[group_number]
            for element in compressedCps[group_number]:
                plt.axvline(x=element, color=clr)
        for minVal in cpData.getCompressedChangePoints():
            plt.axvline(x=minVal, color='k')

        # for xc in cps_0:
        #     plt.axvline(x=xc, color='m') # magenta

        # for xc in cps_1:
        #     plt.axvline(x=xc, color='c') # cyan

        # for xc in cps_2:
        #     plt.axvline(x=xc, color='y') # yellow

        # for xc in cps_3:
        #     plt.axvline(x=xc, color='k') # black

        # for xc in cps_4:
        #     plt.axvline(x=xc, color='#ff9933') # orange
     
        # for xc in cps_5:
        #     plt.axvline(x=xc, color='#aa80ff') # purple

        # for xc in cps_6:
        #     plt.axvline(x=xc, color='#a3c2c2') # greyish- seafoam 

        # key legend 
        lines = []
        lines.append(Line2D([0], [0], color='red', linestyle='--'))
        lines.append(Line2D([0], [0], color='red', linestyle='-'))
        lines.append(Line2D([0], [0], color='blue', linestyle='--'))
        lines.append(Line2D([0], [0], color='blue', linestyle='-'))
        lines.append(Line2D([0], [0], color='green', linestyle='--'))
        lines.append(Line2D([0], [0], color='green', linestyle='-'))
        lines.append(Line2D([0], [0], color='green', linestyle='-.'))
        # lines.append(Line2D([0], [0], color='magenta', linestyle='-'))
        # lines.append(Line2D([0], [0], color='cyan', linestyle='-'))
        # lines.append(Line2D([0], [0], color='yellow', linestyle='-'))
        # lines.append(Line2D([0], [0], color='black', linestyle='-'))
        # lines.append(Line2D([0], [0], color='#ff9933', linestyle='-'))
        # lines.append(Line2D([0], [0], color='#aa80ff', linestyle='-'))
        # lines.append(Line2D([0], [0], color='#a3c2c2', linestyle='-'))


        labels = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
                  # 'left_e0 change point', 'left_e1 change point', 'left_s0 change point', 'left_s1 change point', 'left_w0 change point', 'left_w1 change point', 'left_w2 change point']


        plt.legend(lines, labels)

        # plt.legend(loc='upper left')
        plt.show()



    elif bagName == 'predicate':

        cps_0 = cpData.getTrajectoryBasedPoints('traj_0')
        # cps_1 = cpData.getTrajectoryBasedPoints('traj_1')
        # cps_2 = cpData.getTrajectoryBasedPoints('traj_2')
        # cps_3 = cpData.getTrajectoryBasedPoints('traj_3')
        # cps_4 = cpData.getTrajectoryBasedPoints('traj_4')
        # cps_5 = cpData.getTrajectoryBasedPoints('traj_5')

        plt.plot(bagData[0], 'r--', label='l_btn')
        # plt.plot(bagData[1], 'r1', label='obj_z_loc')
        # plt.plot(bagData[2], 'b--', label='obj_y_loc')
        # plt.plot(bagData[3], 'b1', label='l_btn_pressed')
        # plt.plot(bagData[4], 'g--', label='r_btn_pressed')
        # plt.plot(bagData[5], 'g1', label='obj_vis')

        colrs = ['m', 'c', 'y', '#a3c2c2', '#ff9933', '#aa80ff']        
        compressedCps = cpData.getGroupedData()

        # colrs = ['m', 'c', 'y', 'k', '#ff9933', '#aa80ff', '#a3c2c2']
        for group_number in range(len(compressedCps) - 1):
            clr = colrs[group_number]
            for element in compressedCps[group_number]:
                plt.axvline(x=element, color=clr)
        for minVal in cpData.getCompressedChangePoints():
            plt.axvline(x=minVal, color='k')

        # key legend 
        # lines = []
        # lines.append(Line2D([0], [0], color='red', linestyle='--'))
        # lines.append(Line2D([0], [0], color='red', linestyle='-'))
        # lines.append(Line2D([0], [0], color='blue', linestyle='--'))
        # lines.append(Line2D([0], [0], color='blue', linestyle='-'))
        # lines.append(Line2D([0], [0], color='green', linestyle='--'))
        # lines.append(Line2D([0], [0], color='green', linestyle='-'))

        # labels = ["obj_x_loc", "obj_z_loc", "obj_y_loc"]
         # "l_btn_pressed", "r_btn_pressed", "obj_vis"]

        # plt.legend(lines, labels)
        plt.show()

    elif bagName == "leftGripper":

        cps_0 = cpData.getTrajectoryBasedPoints('traj_0')
        cps_1 = cpData.getTrajectoryBasedPoints('traj_1')
        cps_2 = cpData.getTrajectoryBasedPoints('traj_2')

        plt.plot(bagData[0], 'r--', label='x')
        plt.plot(bagData[1], 'g', label='y')
        plt.plot(bagData[2], 'b--', label='z')


        colrs = ['m', 'c', 'y', '#a3c2c2', '#ff9933', '#aa80ff']        
        compressedCps = cpData.getGroupedData()

        for group_number in range(len(compressedCps) - 1):
            clr = colrs[group_number % (len(colrs)-1)]
            for element in compressedCps[group_number]:
                plt.axvline(x=element, color=clr)
        for minVal in cpData.getCompressedChangePoints():
            plt.axvline(x=minVal, color='k')

        plt.show()
    else:
        print("No data passed")

# should just save it so the stuff can continue 