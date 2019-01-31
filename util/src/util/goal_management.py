

def goalAccomplished(goalList, currentState):
	numGoalsAccomplished = 0
	for goal in goalList:
		if goal in currentState:
			numGoalsAccomplished = numGoalsAccomplished + 1
	if numGoalsAccomplished == len(goalList):
		return True
	return False


# def findAllCombos(objects, action):

def isViable(action):

	# remove actions with no effects
    return True



# def createAction(self, name, origAction, args, preconds, effects, srvFile, params):

#     theOGaction = self.getAction(origAction)
    
#     newActionName = name
#     newActionVars, newActionPreconds, newActionEffects = pddlActionKBFormat(theOGaction.getArgs(), args, preconds, effects)
#     newActionSrvFile = srvFile
#     newActionParams = params
#     newAction = Action(newActionName, newActionVars, newActionPreconds, newActionEffects, newActionSrvFile, newActionParams)
    
