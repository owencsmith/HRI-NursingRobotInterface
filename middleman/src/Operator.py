import rospy
from std_msgs.msg import *
from middleman.msg import Robot, RobotArr, TaskMsg, TaskMsgArr

class Operator:
    def __init__(self, operatorID):
        self.operatorIsBusy = False
        self.needHelpingRobot = False
        self.currentRobot = None
        self.currentHelpingRobot = None
        self.heartBeat = False
        self.operatorID = operatorID
        self.sendNewRobotToOperator = rospy.Publisher('/' + self.operatorID + '/new_robot', Robot, queue_size=10)
        self.robotIsAvailableForExtraViews = rospy.Publisher('/' + self.operatorID + '/robot_is_available_for_extra_views', Bool,
                                                             queue_size=10)
        self.TellOperatorThatRobotCameraIsAvailable = rospy.Publisher('/' + self.operatorID + '/robotForExtraCamera', String, queue_size=10)

        # Set and check heartbeat
        self.checkHeartBeatTimer = rospy.Timer(rospy.Duration(5), self.checkHeartBeat)
        self.setHeartBeatSubscriber = rospy.Subscriber("/" + self.operatorID + "/heartbeat", String, self.setHeartBeat)

        # Tell the middleman to kill me after 2.5 seconds of no heartbeat. Operator publishes its own ID onto /operator/seppuku
        self.seppukuPublisher = rospy.Publisher('/operator/seppuku', String, queue_size=10)
        rospy.sleep(1)
        pass

    def checkHeartBeat(self, data):
        if self.heartBeat:
            self.heartBeat = False
        else:
            self.seppukuPublisher.publish(self.operatorID)
            self.checkHeartBeatTimer.shutdown()
        pass

    def setHeartBeat(self, data):
        self.heartBeat = True
        pass

    def linkToRobot(self, robot):
        robot.operatorID = self.operatorID
        self.currentRobot = robot

    def unlinkFromRobot(self, robot):
        robot.operatorID = ""
        self.currentRobot = None

    def linkToHelpingRobot(self, helpingRobot):
        helpingRobot.operatorID = self.operatorID
        self.currentHelpingRobot = helpingRobot

    def unlinkFromHelpingRobot(self, helpingRobot):
        helpingRobot.operatorID = ""
        self.currentHelpingRobot = None
