import time
from middleman.msg import TaskMsg
class Task:
    def __init__(self, taskName, basePriority, robotName, X, Y, yaw, isPriorityRaised, supervisorID, variables = ""):
        self.taskName = taskName
        self.addedToQueue = int(time.time())
        self.priority_ = basePriority
        # contain extra information needed by tasks
        self.variables = variables
        self.X = X
        self.Y = Y
        self.yaw = yaw
        self.robotName = robotName
        self.priorityRaised = isPriorityRaised
        self.OGSupervisorID = supervisorID

    def getPriority(self):
        timeElapsed_mins = (time.time()-self.addedToQueue) / 60
        if(self.priorityRaised):
            return timeElapsed_mins * (self.priority_ * 1.5)
        return timeElapsed_mins * self.priority_

    def getID(self):
        return str(self.addedToQueue) + "_" + str(self.X) + "_" + str(self.Y)

    # I think this will work?
    def convertTaskMsgToTask(self, taskMsg):
        #calculate basePriority
        timeElapsed = (time.time()-taskMsg.timeAdded) / 60
        if(taskMsg.isRaisedPriority == True):
            basePriority =  round(int(taskMsg.taskPriority/(timeElapsed*1.5)))
        else:
            basePriority = round(int(taskMsg.taskPriority / timeElapsed))

        return Task(taskMsg.taskName, basePriority, taskMsg.robotName,
                    taskMsg.X, taskMsg.Y, taskMsg.yaw, taskMsg.isRaisedPriority, taskMsg.OGSupervisorID,
                    taskMsg.variables)

    def convertTaskToTaskMsg(self):
        taskMsg = TaskMsg()
        taskMsg.taskName = self.taskName
        taskMsg.robotName = self.robotName
        taskMsg.ID = self.getID()
        taskMsg.X = self.X
        taskMsg.Y = self.Y
        taskMsg.yaw = self.yaw
        taskMsg.taskPriority = self.getPriority()
        taskMsg.variables = self.variables
        taskMsg.isRaisedPriority = self.priorityRaised
        taskMsg.timeAdded = self.addedToQueue
        taskMsg.OGSupervisorID = self.OGSupervisorID

        return taskMsg
