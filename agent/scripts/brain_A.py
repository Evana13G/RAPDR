#!/usr/bin/env python

import rospy
import time

from agent.srv import *
from action_primitive_variation.srv import *
from environment.srv import *
from pddl.srv import *
from pddl.msg import *
from util.knowledge_base import KnowledgeBase
from util.data_conversion import * 
from util.goal_management import *
from util.file_io import deleteAllPddlFiles, deleteAllAPVFiles, processLogData
import logging
import random
from std_msgs.msg import (
    Header,
    Empty,
)


def handle_trial(req):
    brainFilePath = os.path.dirname(os.path.realpath(__file__))
    resultsDir = generateResultsDir(brainFilePath, req.runName)
    logFilePath = resultsDir + 'output.txt'
    logData =[]

    env = rospy.ServiceProxy('init_environment', HandleEnvironmentSrv)

    KB = KnowledgeBase()
    lPA = PhysicalAgent('left_gripper')
    rPA = PhysicalAgent('right_gripper')
    
    logData.append("\n################################################################################")
    logData.append("################################################################################")
    logData.append('## Action Primivitive Discovery in Robotic Agents through Action Segmentation ##')
    logData.append('## -- a proof of concept model for knowledge aquisition in intelligent agents ##')
    logData.append('## -- Evana Gizzi, Mateo Guaman Castro, Jivko Sinapov, 2018                   ##')
    logData.append("################################################################################")
    logData.append("################################################################################")
    
    attemptsTime = []
    totalTimeStart = 0

    try:
        # Services
        logData.append('\n ... Setting up services')
        planGenerator = rospy.ServiceProxy('plan_generator_srv', PlanGeneratorSrv)
        planExecutor = rospy.ServiceProxy('plan_executor_srv', PlanExecutorSrv)
        APV = rospy.ServiceProxy('APV_srv', APVSrv)
        partialActionExecutor = rospy.ServiceProxy('partial_plan_executor_srv', PartialPlanExecutorSrv)
        scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)

        logData.append(' ... Cleaning up data from last run')
        deleteAllPddlFiles()
        deleteAllAPVFiles()

        logData.append(' ... Setting up initial data')
        task = 'APD'

        currentState = scenarioData()
        goalLoc = poseStampedToString(getPredicateLocation(currentState.predicateList.predicates, 'at', 'right_gripper'))
        goal1 = '(obtained block)'
        goal = [goal1]
        mode = ['diffsOnly', 'noLoc']
        algoMode = 'APV'
        newPrims = []
        gripperExecutingNewPrim = 'left'
        gripperExecutionValidity = True

        logData.append('\nAgent has the following goal: ')
        logData.append(str(goal))
        logData.append('\nAgent will attempt to accomplish this goal. If attempt fails, agent will try')
        logData.append('to find new actions and replan with those actions. Process repeats until the ')
        logData.append('agent is able to accomplish its goal....')
        attempt = 1

        totalTimeStart = rospy.get_time()

        while(goalAccomplished(goal, currentState.init) == False):
            trialStart = rospy.get_time()
            logData.append('\n***************************   ATTEMPT #' + str(attempt) + '   ***************************')
            logData.append('Setting up domain and problem for attempt #' + str(attempt))

            #####################################################################################
            domainDict = KB.getDomainData()
            domainName = domainDict['domain']
            types = domainDict['types']
            predicates = domainDict['predicates']
            requirements = domainDict['requirements']
            actions = domainDict['actions']
            # print(KB.getDomainData())
            logData.append(' -- Domain setup complete')

            #####################################################################################
            currentState = scenarioData()
            additionalLocations = domainDict['pddlLocs']
            
            initObjs = pddlObjects(currentState.predicateList.predicates, False)
            newPts = copy.deepcopy(initObjs['waypoint'])
            for loc in additionalLocations:
                newPts.append(loc)
            newPts.append(goalLoc)
            newPts = list(set(newPts))
            initObjs['waypoint'] = newPts
            objs = pddlObjectsStringFormat_fromDict(initObjs)
            init = currentState.init
            domain = Domain(domainName, requirements, types, predicates, actions)
            problem = Problem(task, domainName, objs, init, goal)
            filename = task + '_' + str(attempt)
            logData.append(' -- Problem setup complete')

            #####################################################################################
            logData.append('\nTriggering plan generation and execution for attempt #' + str(attempt))
            plan = planGenerator(domain, problem, filename, KB.getActionsLocs())
            logData.append(' -- Plan generation complete')
        
            # executionSuccess = planExecutor(plan.plan)
            # Just to gaurantee we go into APV mode for testing 

            if plan.plan.actions[0].params[0] == gripperExecutingNewPrim:
                gripperExecutionValidity = False
                executionSuccess = 0
            else:
                gripperExecutionValidity = True
                executionSuccess = planExecutor(plan.plan)

            #####################################################################################
            currentState = scenarioData()
            if (executionSuccess == 1):
                logData.append(' -- Plan execution complete')
                if (goalAccomplished(goal, currentState.init) == False):
                    logData.append(' ---- Goal COMPLETE ')
                    break
            else:
                logData.append(' -- Plan execution failed')
                moveLeftArmToStart(lPA)
                moveRightArmToStart(rPA)
            #####################################################################################

            # MODE 1: start
            if algoMode == 'APV':
            # MODE 1: end

                print('\nGenerating all possible action/arg combinations (to send to APV) for attempt #' + str(attempt))
                momentOfFailurePreds = scenarioData().predicates
                APVtrials = generateAllCombos()
                    
                logData.append(' -- generation complete, ' + str(len(APVtrials)) + ' total combos found')
                #for t in APVtrials:
                ###    print(t)

            #####################################################################################
                logData.append('\nFinding segmentation possibilities (across all combos generated) for attempt #' + str(attempt))
                trialNo = 0


                # MODE 1: start
                while(len(APVtrials) >= 1): 
                # MODE 1: end 
                # Pretty much remove this p for MODE 2 

                    comboChoice = random.randint(0, len(APVtrials) - 1)
                    logData.append("\n -- Combo # " + str(trialNo) + ': ' + str(APVtrials[comboChoice]))

                    try:
                        #### Find change points    
                        resp = APV(APVtrials[comboChoice][0], APVtrials[comboChoice][1], APVtrials[comboChoice][2], APVtrials[comboChoice][3], req.clusterThreshold, req.minClusterSize)
                        logData.append(' ---- ' + str(len(resp.endEffectorInfo)) + " total change points found")
                        logData.append("Trying partial plan execution on segmentations")
                        moveLeftArmToStart(lPA)
                        moveRightArmToStart(rPA)
                        #### Iterate across segmentations
                        i = 0
                        while i <= len(resp.endEffectorInfo) - 2:
                            # print(" ---- starting iteration #" + str(i+1))
                            startingState = scenarioData().predicateList
                            resp_2 = partialActionExecutor(APVtrials[comboChoice][1], resp.endEffectorInfo[i], resp.endEffectorInfo[i+1])
                            time.sleep(2)
                            endingState = scenarioData().predicateList

                            ##### Here is where you decide what gets added 
                            if(resp_2.success_bool == 1):
                                logData.append(' -- iteration ' + str(i) + ' successful!')

                                new_name = "action_attempt_" + str(attempt) + '_trial' + str(trialNo) + '_seg' + str(i) 
                                            #'.' + poseStampedToString(resp.endEffectorInfo[i]) + 
                                            #'.' + poseStampedToString(resp.endEffectorInfo[i+1])
                                orig_name = APVtrials[comboChoice][0]
                                orig_args = [APVtrials[comboChoice][1], APVtrials[comboChoice][2], APVtrials[comboChoice][3]]
                                gripperData = [resp.endEffectorInfo[i], resp.endEffectorInfo[i+1]]
                                gripper = orig_args[0]

                                newAction = KB.createAction(new_name, 
                                                            orig_name, 
                                                            orig_args,
                                                            startingState, 
                                                            endingState, 
                                                            PartialPlanExecutorSrv, 
                                                            gripper,
                                                            gripperData, 
                                                            mode)

                                if isViable(newAction):
                                    logData.append(' ---- Segmentation VIABLE! Adding to knowledge base')
                                    KB.addAction(newAction)
                                    newPrims.append(newAction)
                            else:
                                logData.append(' -- iteration ' + str(i) + ' not successful')
                            i = i + 1 
                    except rospy.ServiceException, e:
                        logData.append("Service call failed: %s"%e)
                    del APVtrials[comboChoice]
                    trialNo = trialNo + 1 
                # MODE 1: start
                algoMode = 'planAndRun'               
            else:
                if newPrims == []:
                    logData.append('No Prims to Execute')
                    algoMode = 'APV' 
                else:
                # MODE 1: end
                # Pretty much, can remove this for MODE 2

                    if gripperExecutionValidity == True:
                        logData.append('Executing a Primitive')
                        if len(newPrims) == 1:
                            actionIndex = 0
                            actionToExecute = newPrims[actionIndex]
                        else:
                            actionIndex = random.randint(0, len(newPrims)-1)
                            actionToExecute = newPrims[actionIndex] 
                        resp_3 = partialActionExecutor(actionToExecute.getGripper(), actionToExecute.getExecutionParams()[0], actionToExecute.getExecutionParams()[1])
                        gripperExecutingNewPrim = actionToExecute.getGripper()
                        del newPrims[actionIndex]

            currentState = scenarioData()
            trialEnd = rospy.get_time()
            attemptsTime.append(trialEnd - trialStart)
            attempt = attempt + 1

        logData.append('\nGoal accomplished!')

        totalTimeEnd = rospy.get_time()
        logData.append("\nTimes for each trial (in s): ")
        logData.append(str(attemptsTime))
        logData.append("Total time elapsed: " + str(totalTimeEnd - totalTimeStart))

        compileResults(brainFilePath, req.runName)
        processLogData(logFilePath, logData)        
        return BrainSrvResponse(attemptsTime, totalTimeEnd - totalTimeStart) 
    
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        totalTimeEnd = rospy.get_time()
        processLogData(logFilePath, logData)        
        return BrainSrvResponse(attemptsTime, totalTimeEnd - totalTimeStart) 
    


def main():
    rospy.init_node("agent_brain_A")
    rospy.wait_for_message("/robot/sim/started", Empty)

    rospy.wait_for_service('APV_srv', timeout=60)
    rospy.wait_for_service('partial_plan_executor_srv', timeout=60)
    rospy.wait_for_service('scenario_data_srv', timeout=60)
    rospy.wait_for_service('plan_generator_srv', timeout=60)
    rospy.wait_for_service('plan_executor_srv', timeout=60)
    rospy.wait_for_service('press_button_srv', timeout=60)
    rospy.wait_for_service('obtain_object_srv', timeout=60)


    s = rospy.Service("brain_A_srv", BrainSrv, handle_trial)

    rospy.spin()

    return 0 


if __name__ == "__main__":
    main()
