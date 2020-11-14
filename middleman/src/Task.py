import time
from middleman.msg import TaskMsg
class Task:
    def __init__(self, taskName, basePriority, robotName, X, Y, isPriorityRaised, variables):
        self.taskName = taskName
        self.addedToQueue = time.time()
        self.priority_ = basePriority
        # contain extra information needed by tasks
        self.variables = variables
        self.X = X
        self.Y = Y
        self.robotName = robotName
        self.priorityRaised = isPriorityRaised

    def getPriority(self):
        timeElapsed_mins = (time.time()-self.addedToQueue) / 60
        if(self.priorityRaised):
            return timeElapsed_mins * (self.priority_ * 1.5)
        return timeElapsed_mins * self.priority_

    def getID(self):
        return str(self.addedToQueue) + "_" + str(self.X) + "_" + str(self.Y)

    def convertTaskToTaskMsg(self):
        taskMsg = TaskMsg()
        taskMsg.taskName = self.taskName
        taskMsg.robotName = self.robotName
        taskMsg.ID = self.getID()
        taskMsg.X = self.X
        taskMsg.Y = self.Y
        taskMsg.taskPriority = self.getPriority()
        taskMsg.variables = self.variables
        taskMsg.isRaisedPriority = self.priorityRaised

        return taskMsg
