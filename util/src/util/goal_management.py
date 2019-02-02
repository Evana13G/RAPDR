

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
    if action.getEffects() == []:
        return False
    return True

