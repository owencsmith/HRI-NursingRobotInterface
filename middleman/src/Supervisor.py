import rospy
from std_msgs.msg import *
from middleman.msg import Robot, RobotArr, TaskMsg, TaskMsgArr

class Supervisor:
    def __init__(self, supervisorID):
        self.supervisorID = supervisorID

        self.activeRobotDictionary = {}
        self.activeTaskList = []
        self.taskPriorityQueue = []

        self.statePublisherForSupervisor = rospy.Publisher('/' + supervisorID + '/robotState', RobotArr, queue_size=10)
        self.taskListPublisher = rospy.Publisher('/' + supervisorID + '/taskList', TaskMsgArr, queue_size=10)
        self.taskReassignmentPublisher = rospy.Publisher('/' + supervisorID + '/taskReassignment', TaskMsgArr, queue_size=10)

        # Set and check heartbeat
        self.checkHeartBeatTimer = rospy.Timer(rospy.Duration(2.5), self.checkHeartBeat)
        self.setHeartBeatSubscriber = rospy.Subscriber("/" + self.supervisorID + "/heartbeat", String, self.setHeartBeat)

        # Tell the middleman to kill me after 2.5 seconds of no heartbeat. supervisor publishes its own ID onto /operator/seppuku
        self.seppukuPublisher = rospy.Publisher('/supervisor/seppuku', String, queue_size=10)
        rospy.sleep(1)
        pass

    def checkHeartBeat(self, data):
        if self.heartBeat:
            self.heartBeat = False
        else:
            self.seppukuPublisher.publish(self.supervisorID)
            self.checkHeartBeatTimer.shutdown()
        pass

    def setHeartBeat(self, data):
        self.heartBeat = True
        pass


    # need linking methods