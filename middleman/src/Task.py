import time
class Task:
    def __init__(self, taskName, basePriority, robotName, X, Y, variables):
        self.taskName = taskName
        self.addedToQueue_ = time.time()
        self.priority_ = basePriority
        # contain extra information needed by tasks
        self.variables = variables
        self.X = X
        self.Y = Y
        self.robotName = robotName

    def getPriority(self):
        timeElapsed_mins = (time.time()-self.addedToQueue_) / 60
        return timeElapsed_mins * self.priority_
