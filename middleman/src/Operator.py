import rospy
from std_msgs.msg import *
from middleman.msg import Robot, RobotArr, TaskMsg, TaskMsgArr

class Operator:
    def __init__(self, operatorID):
        self.operatorIsBusy = False
        self.needHelpingRobot = False
        self.currentRobot = None
        self.operatorID = operatorID
        self.sendNewRobotToOperator = rospy.Publisher('/' + self.operatorID + '/new_robot', Robot, queue_size=10)
        self.robotIsAvailableForExtraViews = rospy.Publisher('/' + self.operatorID + '/robot_is_available_for_extra_views', Bool,
                                                             queue_size=10)
        self.TellOperatorThatRobotCameraIsAvailable = rospy.Publisher('/' + self.operatorID + '/robotForExtraCamera', String, queue_size=10)

        pass
