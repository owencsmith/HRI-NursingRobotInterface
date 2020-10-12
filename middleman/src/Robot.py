import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

class Robot():
    def __init__(self, name, pose):
        self.name = name
        self.currentTask = "IDLE"
        self.status = "OK" # Good (OK), Bad (NOK), Operator Control (OPC)
        self.pose = pose
        #subscribe to amcl to get pose estimate
        rospy.Subscriber('amcl_pose', geometry_msgs/PoseWithCovarianceStamped, self.updateRobotPose)
        #publishers
        self.statePublisherForOperator = rospy.Publisher('/operator/robotState', String, queue_size=10)
        self.statePublisherForSupervisor = rospy.Publisher('/supervisor/robotState', String, queue_size=10)

    def updateRobotPose(self, data):
        self.pose = data.pose

    # idk how to implement this cleanly but we , maybe publish yourself? that encapsulates everything
    def publish(self):
        print("Published specific robot's state")
        pass