import time


class Task:

    def __init__(self, name, basePriority):
        self.name = name
        self.addedToQueue_ = time.time()
        self.priority_ = basePriority

    def getPriority(self):
        timeElapsed_mins = (time.time()-self.addedToQueue_) % 60
        return timeElapsed_mins * self.priority_
