import time
class Task:
    def __init__(self, taskName, basePriority, robotName = "unassigned"):
        self.taskName = taskName
        self.addedToQueue_ = time.time()
        self.priority_ = basePriority
        if(robotName is not "unassigned"):
            self.robotName = robotName

    def getPriority(self):
        timeElapsed_mins = (time.time()-self.addedToQueue_) / 60
        return timeElapsed_mins * self.priority_
