#!/usr/bin/env python


###### put a goal 
###### get a pddl plan for it 
###### try to execute it
###### if fails, get predicates list, and launch APV for each action until success
###### using the change points, test the actions and fire off the predicates list, storing all 
###### try each of the actions, return if successful
######analyze the new actions


import rospy

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
        goal = ['(is_visible block)']
        currentState = scenarioData()

        mode = ['diffsOnly', 'noLoc']

        print('\nAgent has the following goal: ')
        print(goal)
        print('\nAgent will attempt to accomplish this goal. If attempt fails, agent will try')
        print('to find new actions and replan with those actions. Process repeats until the ')
        print('agent is able to accomplish its goal....')
        attempt = 1

        while(goalAccomplished(goal, currentState.init) == False):
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
            #if attempt == 1:
            #    executionSuccess = 0
            #else:
            
            executionSuccess = planExecutor(plan.plan)

            #####################################################################################
            if (executionSuccess == 1):
                print(' -- Plan execution complete: Goal accomplished!')
                # Maybe check the goal?
            else:
                print(' -- Plan execution complete: Goal NOT accomplished')
                # Should prob break this into a diff module... findNewAction module 
                # Do one that can for each action, return new sub actoins to try ?

            #####################################################################################
                print('\nGenerating all possible action/arg combinations (to send to APV) for attempt #' + str(attempt))
                momentOfFailurePreds = scenarioData().predicates
                APVtrials = []
                ##### Here is where you decide what to iterate over
                objectsToIterate = pddlObjects(currentState.predicateList.predicates, False)
                for action in KB.getActions():

                    args = action.getNonLocationVars()
                    actionTrials = []
                    actionTrials.append(action.getName())
                    for arg in args:
                        itemsChoices = objectsToIterate[arg]
                        choice = itemsChoices[random.randint(0, len(itemsChoices) - 1)]
                        actionTrials.append(choice)
                    if len(args) < 4:
                        actionTrials.append(None)
                    APVtrials.append(actionTrials)
                print(' -- generation complete, ' + str(len(APVtrials)) + ' total combos found')

            #####################################################################################
                print('\nFinding segmentation possibilities (across all combos generated) for attempt #' + str(attempt))
                trialNo = 0
                while(trialNo < len(APVtrials)): # Change this to be stochastic selection
                    print(" -- Combo # " + str(trialNo) + ': ' + str(APVtrials[trialNo]))

                    if (APVtrials[trialNo][0] == 'press_button') and (APVtrials[trialNo][2] == 'left_button'):

                        try:
                            #### Find change points
                            resp = APV(APVtrials[trialNo][0], APVtrials[trialNo][1], APVtrials[trialNo][2], APVtrials[trialNo][3])
                            print(' ---- ' + str(len(resp.endEffectorInfo)) + " total change points found")
                            print("\n Trying partial plan execution on segmentations")

                            #### Iterate across segmentations
                            i = 0
                            while i <= len(resp.endEffectorInfo) - 2:
                                # print(" ---- starting iteration #" + str(i+1))
                                startingState = scenarioData().predicateList
                                resp_2 = partialActionExecutor(APVtrials[trialNo][1], resp.endEffectorInfo[i], resp.endEffectorInfo[i+1])
                                sleep(1)
                                endingState = scenarioData().predicateList

                                ##### Here is where you decide what gets added 
                                if(resp_2.success_bool == 1):
                                    print(' -- iteration ' + str(i) + ' successful!')

                                    new_name = "action_attempt_" + str(attempt) + '_trial' + str(trialNo) + '_seg' + str(i) 
                                                #'.' + poseStampedToString(resp.endEffectorInfo[i]) + 
                                                #'.' + poseStampedToString(resp.endEffectorInfo[i+1])
                                    orig_name = APVtrials[trialNo][0]
                                    orig_args = [APVtrials[trialNo][1], APVtrials[trialNo][2], APVtrials[trialNo][3]]
                                    gripperData = [resp.endEffectorInfo[i], resp.endEffectorInfo[i+1]]

                                    newActionData = {}
                                    newActionData['name'] = new_name
                                    newActionData['orig_name'] = orig_name
                                    newActionData['orig_args'] = orig_args
                                    newActionData['preconditions'] = startingState
                                    newActionData['effects'] = endingState
                                    newActionData['srvFile'] = PartialPlanExecutorSrv
                                    newActionData['params'] = gripperData

                                    newAction = KB.createAction(new_name, 
                                                                orig_name, 
                                                                orig_args,
                                                                startingState, 
                                                                endingState, 
                                                                PartialPlanExecutorSrv, 
                                                                gripperData, 
                                                                mode)

                                    if isViable(newAction):
                                        print(' ---- Segmentation VIABLE! Adding to knowledge base')
                                        KB.addAction(newAction)

                                else:
                                    print(' -- iteration ' + str(i) + ' not successful')
                                i = i + 1 
                        except rospy.ServiceException, e:
                            print("Service call failed: %s"%e)
                    trialNo = trialNo + 1 

            attempt = attempt + 1


    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    main()
