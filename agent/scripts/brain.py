#!/usr/bin/env python


###### put a goal 
###### get a pddl plan for it 
###### try to execute it
###### if fails, get predicates list, and launch APV for each action until success
###### using the change points, test the actions and fire off the predicates list, storing all 
###### try each of the actions, return if successful
######analyze the new actions


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
from util.file_io import deleteAllPddlFiles 

import random
from std_msgs.msg import (
    Header,
    Empty,
)

def main():
    rospy.init_node("agent_brain")
    rospy.wait_for_message("/robot/sim/started", Empty)

    rospy.wait_for_service('APV_srv', timeout=60)
    rospy.wait_for_service('partial_plan_executor_srv', timeout=60)
    rospy.wait_for_service('scenario_data_srv', timeout=60)
    rospy.wait_for_service('plan_generator_srv', timeout=60)
    rospy.wait_for_service('plan_executor_srv', timeout=60)

    KB = KnowledgeBase()

    print("\n################################################################################")
    print("################################################################################")
    print('## Action Primivitive Discovery in Robotic Agents through Action Segmentation ##')
    print('## -- a proof of concept model for knowledge aquisition in intelligent agents ##')
    print('## -- Evana Gizzi, Mateo Guaman Castro, Jivko Sinapov, 2018                   ##')
    print("################################################################################")
    print("################################################################################")
    try:
        # Services
        print('\n ... Setting up services')
        planGenerator = rospy.ServiceProxy('plan_generator_srv', PlanGeneratorSrv)
        planExecutor = rospy.ServiceProxy('plan_executor_srv', PlanExecutorSrv)
        APV = rospy.ServiceProxy('APV_srv', APVSrv)
        partialActionExecutor = rospy.ServiceProxy('partial_plan_executor_srv', PartialPlanExecutorSrv)
        scenarioData = rospy.ServiceProxy('scenario_data_srv', ScenarioDataSrv)

        print(' ... Cleaning up data from last run')
        deleteAllPddlFiles()

        #####################################################################################
        #                                                                                   #
        #    This is where the highest level of the main algorithm should be implemented    #
        #                                                                                   #
        #    Domain:                     Problem:                                           #
        #    string name                 string task                                        #
        #    string[] requirements       string domain                                      #
        #    string[] types              string[] objects                                   #
        #    string[] predicates         string[] init                                      #
        #    string[] actions            string goal                                        #
        #                                                                                   #
        #####################################################################################
        print(' ... Setting up initial data')
        task = 'APD'


        currentState = scenarioData()
        goalLoc = poseStampedToString(getPredicateLocation(currentState.predicateList.predicates, 'at', 'right_gripper'))
        goal1 = '(obtained block)'
        # goal2 = '(obj_at block ' + goalLoc  + ')'
        goal = [goal1]
        mode = ['diffsOnly', 'noLoc']
        algoMode = 'APV'
        newPrims = []
        gripperExecutingNewPrim = 'left'
        gripperExecutionValidity = True
        #mode = ['diffsOnly']

        print('\nAgent has the following goal: ')
        print(goal)
        print('\nAgent will attempt to accomplish this goal. If attempt fails, agent will try')
        print('to find new actions and replan with those actions. Process repeats until the ')
        print('agent is able to accomplish its goal....')
        attempt = 1
        attemptsTime = []
        totalTimeStart = time.time()
        while(goalAccomplished(goal, currentState.init) == False):
            trialStart = time.time()
            print('\n***************************   ATTEMPT #' + str(attempt) + '   ***************************')
            print('Setting up domain and problem for attempt #' + str(attempt))

            #####################################################################################
            domainDict = KB.getDomainData()
            domainName = domainDict['domain']
            types = domainDict['types']
            predicates = domainDict['predicates']
            requirements = domainDict['requirements']
            actions = domainDict['actions']
            # print(KB.getDomainData())
            print(' -- Domain setup complete')

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
            print(' -- Problem setup complete')

            #####################################################################################
            print('\nTriggering plan generation and execution for attempt #' + str(attempt))
            plan = planGenerator(domain, problem, filename, KB.getActionsLocs())
            print(' -- Plan generation complete')
        
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
                print(' -- Plan execution complete')
                if (goalAccomplished(goal, currentState.init) == False):
                    print(' ---- Goal COMPLETE ')
                    break
            else:
                print(' -- Plan execution failed')
                moveLeftArmToStart()
                moveRightArmToStart()


                # Should prob break this into a diff module... findNewAction module 
                # Do one that can for each action, return new sub actoins to try ?

            #####################################################################################

            # MODE 1: start
            if algoMode == 'APV':
            # MODE 1: end


            # MODE 2: start
            # if newPrims == []:
            # MODE 2: end

                print('\nGenerating all possible action/arg combinations (to send to APV) for attempt #' + str(attempt))
                momentOfFailurePreds = scenarioData().predicates
                APVtrials = []
                
                APVtrials.append(['obtain_object', 'left_gripper', 'wall', None]) 
                APVtrials.append(['obtain_object', 'left_gripper', 'table', None]) 
                APVtrials.append(['obtain_object', 'left_gripper', 'block', None]) 
                APVtrials.append(['press_button', 'left_gripper', 'left_button', None]) 
                APVtrials.append(['press_button', 'right_gripper', 'left_button', None]) 

                
                ##### BOTH need this #####
                #objectsToIterate = pddlObjects(currentState.predicateList.predicates, False)
                #for action in KB.getActions():#

                    ############ UNDER CONSTRUCTION ############
                #    args = action.getNonLocationVars()
                #    actionTrials = []
                #    actionTrial = []
                #    actionTrial.append(action.getName())
                #    actionTrial.append('left_gripper')
                #    actionTrials.append(actionTrial)
                #    lenTrials = len(actionTrials)
                #    newTrials = [] 
                #    for i in range(len(args)-1):
                #        for j in range(lenTrials):
                #            for argChoice in objectsToIterate[args[i+1]]:
                #                newTrial = copy.deepcopy(actionTrials[j])
                #                newTrial.append(argChoice)
                #                newTrials.append(newTrial)
                #        actionTrials = newTrials
                #    APVtrials.append(actionTrials)

                #addNones = copy.deepcopy(APVtrials)
                #replaceAPV = []
                #for trial in addNones:
                    #new = trial.append(None)
                #    replaceAPV.append(trial.append(None))
                #APVtrials = replaceAPV


                ############################################
                    
                print(' -- generation complete, ' + str(len(APVtrials)) + ' total combos found')
                #for t in APVtrials:
                ###    print(t)

            #####################################################################################
                print('\nFinding segmentation possibilities (across all combos generated) for attempt #' + str(attempt))
                trialNo = 0


                # MODE 1: start
                while(len(APVtrials) >= 1): 
                # MODE 1: end 
                # Pretty much remove this loop for MODE 2 

                    comboChoice = random.randint(0, len(APVtrials) - 1)
                    print(" -- Combo # " + str(trialNo) + ': ' + str(APVtrials[comboChoice]))

                    try:
                        #### Find change points    
                        resp = APV(APVtrials[comboChoice][0], APVtrials[comboChoice][1], APVtrials[comboChoice][2], APVtrials[comboChoice][3])
                        print(' ---- ' + str(len(resp.endEffectorInfo)) + " total change points found")
                        print("\n Trying partial plan execution on segmentations")
                        moveLeftArmToStart()
                        moveRightArmToStart()
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
                                print(' -- iteration ' + str(i) + ' successful!')

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
                                    print(' ---- Segmentation VIABLE! Adding to knowledge base')
                                    KB.addAction(newAction)
                                    newPrims.append(newAction)
                            else:
                                print(' -- iteration ' + str(i) + ' not successful')
                            i = i + 1 
                    except rospy.ServiceException, e:
                        print("Service call failed: %s"%e)
                    del APVtrials[comboChoice]
                    trialNo = trialNo + 1 
                # MODE 1: start
                algoMode = 'planAndRun'               
            else:
                if newPrims == []:
                    print('No Prims to Execute')
                    algoMode = 'APV' 
                else:
                # MODE 1: end
                # Pretty much, can remove this for MODE 2

                    if gripperExecutionValidity == True:
                        print('Executing a Primitive')
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
            trialEnd = time.time()
            attemptsTime.append(trialEnd - trialStart)
            attempt = attempt + 1
        print('\nGoal accomplished!')
        totalTimeEnd = time.time()
        print("\nTimes for each trial (in s): ")
        print(attemptsTime);
        print("Total time elapsed: " + str(totalTimeEnd - totalTimeStart))

    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    main()
