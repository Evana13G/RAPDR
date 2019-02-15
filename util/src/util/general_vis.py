#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from util.bayesian_change_point import BayesianChangePoint
from util.ros_bag import RosBag
from util.file_io import * 

def ROSbag_with_CPs(bagName, bagData, cpData):

    imageFilePath = bagName + '.svg'


    if bagName == "leftGripper":

        cps_0 = cpData.getTrajectoryBasedPoints('traj_0')
        cps_1 = cpData.getTrajectoryBasedPoints('traj_1')
        cps_2 = cpData.getTrajectoryBasedPoints('traj_2')

        fig = plt.figure()
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
        
        saveFigureToImage(fig, imageFilePath, 'APV')

    elif bagName == "rightGripper":

        cps_0 = cpData.getTrajectoryBasedPoints('traj_0')
        cps_1 = cpData.getTrajectoryBasedPoints('traj_1')
        cps_2 = cpData.getTrajectoryBasedPoints('traj_2')

        fig = plt.figure()
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
        
        saveFigureToImage(fig, imageFilePath, 'APV')

    else:
        print("No data passed")


    # elif bagName == 'jointState':
    #     cps_0 = cpData.getTrajectoryBasedPoints('traj_0')
    #     cps_1 = cpData.getTrajectoryBasedPoints('traj_1')
    #     cps_2 = cpData.getTrajectoryBasedPoints('traj_2')
    #     cps_3 = cpData.getTrajectoryBasedPoints('traj_3')
    #     cps_4 = cpData.getTrajectoryBasedPoints('traj_4')
    #     cps_5 = cpData.getTrajectoryBasedPoints('traj_5')
    #     cps_6 = cpData.getTrajectoryBasedPoints('traj_6')

    #     fig = plt.figure()
    #     plt.plot(bagData[0], 'r--', label='left_e0')
    #     plt.plot(bagData[1], 'r1', label='left_e1')
    #     plt.plot(bagData[2], 'b--', label='left_s0')
    #     plt.plot(bagData[3], 'b1', label='left_s1')
    #     plt.plot(bagData[4], 'g--', label='left_w0')
    #     plt.plot(bagData[5], 'g1', label='left_w1')
    #     plt.plot(bagData[6], 'g^', label='left_w2')

    #     colrs = ['m', 'c', 'y', '#a3c2c2', '#ff9933', '#aa80ff']        
    #     compressedCps = cpData.getGroupedData()

    #     for group_number in range(len(compressedCps) - 1):
    #         clr = colrs[group_number]
    #         for element in compressedCps[group_number]:
    #             plt.axvline(x=element, color=clr)
    #     for minVal in cpData.getCompressedChangePoints():
    #         plt.axvline(x=minVal, color='k')

    #     # key legend 
    #     lines = []
    #     lines.append(Line2D([0], [0], color='red', linestyle='--'))
    #     lines.append(Line2D([0], [0], color='red', linestyle='-'))
    #     lines.append(Line2D([0], [0], color='blue', linestyle='--'))
    #     lines.append(Line2D([0], [0], color='blue', linestyle='-'))
    #     lines.append(Line2D([0], [0], color='green', linestyle='--'))
    #     lines.append(Line2D([0], [0], color='green', linestyle='-'))
    #     lines.append(Line2D([0], [0], color='green', linestyle='-.'))
    #     # lines.append(Line2D([0], [0], color='magenta', linestyle='-'))
    #     # lines.append(Line2D([0], [0], color='cyan', linestyle='-'))
    #     # lines.append(Line2D([0], [0], color='yellow', linestyle='-'))
    #     # lines.append(Line2D([0], [0], color='black', linestyle='-'))
    #     # lines.append(Line2D([0], [0], color='#ff9933', linestyle='-'))
    #     # lines.append(Line2D([0], [0], color='#aa80ff', linestyle='-'))
    #     # lines.append(Line2D([0], [0], color='#a3c2c2', linestyle='-'))


    #     labels = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    #               # 'left_e0 change point', 'left_e1 change point', 'left_s0 change point', 'left_s1 change point', 'left_w0 change point', 'left_w1 change point', 'left_w2 change point']


    #     plt.legend(lines, labels)
    #     plt.legend(loc='upper left') 
    #     saveFigureToImage(fig, imageFilePath, 'APV')


    # elif bagName == 'predicate':

    #     #### UNDER CONSTRUCTION ####

    #     cps_0 = cpData.getTrajectoryBasedPoints('traj_0')

    #     fig = plt.figure()
    #     plt.plot(bagData[0], 'r--', label='l_btn')
    #     # plt.plot(bagData[1], 'r1', label='obj_z_loc')
    #     # plt.plot(bagData[2], 'b--', label='obj_y_loc')
    #     # plt.plot(bagData[3], 'b1', label='l_btn_pressed')
    #     # plt.plot(bagData[4], 'g--', label='r_btn_pressed')
    #     # plt.plot(bagData[5], 'g1', label='obj_vis')

    #     colrs = ['m', 'c', 'y', '#a3c2c2', '#ff9933', '#aa80ff']        
    #     compressedCps = cpData.getGroupedData()

    #     for group_number in range(len(compressedCps) - 1):
    #         clr = colrs[group_number]
    #         for element in compressedCps[group_number]:
    #             plt.axvline(x=element, color=clr)
    #     for minVal in cpData.getCompressedChangePoints():
    #         plt.axvline(x=minVal, color='k')

    #     saveFigureToImage(fig, imageFilePath, 'APV')

    # elif bagName == "grippers":

    #     cps_0 = cpData.getTrajectoryBasedPoints('traj_0')
    #     cps_1 = cpData.getTrajectoryBasedPoints('traj_1')
    #     cps_2 = cpData.getTrajectoryBasedPoints('traj_2')
    #     cps_3 = cpData.getTrajectoryBasedPoints('traj_3')
    #     cps_4 = cpData.getTrajectoryBasedPoints('traj_4')
    #     cps_5 = cpData.getTrajectoryBasedPoints('traj_5')
    #     cps_6 = cpData.getTrajectoryBasedPoints('traj_6')

    #     fig = plt.figure()
    #     plt.plot(bagData[0], 'r--', label='x')
    #     plt.plot(bagData[1], 'g', label='y')
    #     plt.plot(bagData[2], 'b--', label='z')


    #     colrs = ['m', 'c', 'y', '#a3c2c2', '#ff9933', '#aa80ff']        
    #     compressedCps = cpData.getGroupedData()

    #     for group_number in range(len(compressedCps) - 1):
    #         clr = colrs[group_number % (len(colrs)-1)]
    #         for element in compressedCps[group_number]:
    #             plt.axvline(x=element, color=clr)
    #     for minVal in cpData.getCompressedChangePoints():
    #         plt.axvline(x=minVal, color='k')
        
    #     saveFigureToImage(fig, imageFilePath, 'APV')
