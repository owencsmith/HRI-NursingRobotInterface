#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from middleman.msg import Robot

#TODO: Method to create a new robot and add to dictionary
# if robot is not in dictionary then add it to the dictionary -- Done

#TODO: Pickup ros odom messages to detect new robots
# a robot sends a message that its booted, we pick that up
# Launch a node with each robot that tells the middle man that a new robot exists
# Use the name of the robot to determine all the topics for that specific robot since they are all the same
# except the name

#TODO: We're proabbly going to have to have a topic for each of the robot joints as well.... Robot is going
# to encapsulate a lot of information


class Middleman():
    def __init__(self):
        rospy.init_node("middleman", anonymous = True)
        # dictionary for task codes
        """
        Task codes:
            NAV: Robot is navigating 
            HLP: Robot Operator is asking for another robot for additional camera views
            CLN: Robot is asked to clean
            DLV: Robot is asked to deliver something 
            IDLE: Robot has no current task
            SOS: Supervisor passes control of the robot to the operator
        """
        self.taskCodes = {

        }
        # dictionary of all active robots
        self.activeRobotDictionary = {}

        #operator queue
        self.robotsForOperator = []

        self.rate = rospy.Rate(1)

        #TODO:
        rospy.Subscriber("/supervisor/nav_task", String, self.processNavTask)
        rospy.Subscriber("/supervisor/sos", String, self.passRobotToQueueForOperator)
        rospy.Subscriber("/operator/done_helping", String, self.advanceRobotHelpQueue)
        rospy.Subscriber("/operator/request_extra_views_from_robot", String, self.sendAnotherRobotForCameraViews)
        rospy.Subscriber("/robot/stuck", String, self.passRobotToQueueForOperator)
        rospy.Subscriber("/robot/done_task", String, self.alertSupervisorRobotIsDone)
        rospy.Subscriber("/robot/new_robot_running", String, self.createNewRobot)


        #publishers
        # TODO: Make our own Robot message with fields of robot object -- Done
        # TODO: when operator is done with a robot, pop from list and send new one using Service/Client (not publisher) -- N/A anymore

        self.sendNewRobotToOperator  = rospy.Publisher('/operator/new_robot', Robot, queue_size=10)
        self.robotIsAvailableForExtraViews = rospy.Publisher('/operator/robot_is_available_for_extra_views', Bool, queue_size = 10)
        self.robotsLeftInQueue = rospy.Publisher('/operator/robots_left_in_queue', Int32, queue_size = 10)
        self.robotsLeftInQueue = rospy.Publisher('/supervisor/robots_finished_task', String, queue_size=10)
        self.statePublisherForOperator = rospy.Publisher('/operator/robotState', Robot, queue_size=10)
        self.statePublisherForSupervisor = rospy.Publisher('/supervisor/robotState', Robot, queue_size=10)
        pass


    def processNavTask(self, data):
        print("Processing Nav Goal")
        # parses Robot name XY string and sends to robots movebase
        dataList = data.data.split()
        robotName = dataList[0]
        X = dataList[1]
        Y = dataList[2]
        self.sendRobotToPos(robotName, float(X), float(Y))
        pass

    def sendRobotToPos(self, robotName, X, Y):
        currentRobot = self.activeRobotDictionary[robotName]
        currentRobot.currentTask = "NAV"
        worldCoordinates = self.guiCoordinatesToWorldCoordinates([X, Y])
        # publish coordinates to move_base of specific robot

    def passRobotToQueueForOperator(self, data):
        print("Passing robot")
        #get robot name
        robotName = data.data.split()[0]
        #uses name to get robot object
        robotThatNeedsHelp = self.activeRobotDictionary[robotName]
        #change status to OPC
        #task code doesn't change so that the operator knows whats up
        robotThatNeedsHelp.status = "OPC"
        #add the robot to the operator queue
        self.robotsForOperator.append(robotThatNeedsHelp)

        pass

    def advanceRobotHelpQueue(self,data):
        print("Loading Next Robot")
        #parse robot name
        robotName = data.data.split()[0]
        robotThatWasHelped = self.activeRobotDictionary[robotName]
        #This gets published, no need to update supervisor
        robotThatWasHelped.status = "OK"
        robotThatWasHelped.currentTask = "IDLE"
        # pop another robot from the queue if queue is not empty
        nextRobot = self.robotsForOperator.pop()
        # publish that robot to /operator/new_robot
        self.sendNewRobotToOperator.publish(nextRobot)
        pass

    def sendAnotherRobotForCameraViews(self, data):
        print("Requesting another robot for camera views")
        robotAvailableForHelp = False
        robotThatWillHelp = None
        # Is there an IDLE robot that CAN be used?
        for robot in self.activeRobotDictionary.values():
            if(robot.currentTask == "IDLE"):
                # find it and grab it from the dictionary
                robot.currentTask = "HLP"
                # publish that a robot is on its way
                robot.status = "OPC"
                robotAvailableForHelp = True
                robotThatWillHelp = robot
                break
        if(not robotAvailableForHelp):
            return False
        else:
            # parse the string for the robot to navigate to for more views
            robotToNavigateTo = self.activeRobotDictionary[data.data.split()[0]]
            # grab the pose of the robot to navigate to
            robotToNavigateToPose = robotToNavigateTo.pose
            robotToNavigateToX = robotToNavigateToPose.pose.pose.position.x
            robotToNavigateToY = robotToNavigateToPose.pose.pose.position.y
            # Tell the IDLE robot to navigate to the robot that needs more views
            self.sendRobotToPos(str(robotThatWillHelp.name), robotToNavigateToX, robotToNavigateToY)
        pass

    def guiCoordinatesToWorldCoordinates(self, coordinates):
        pass

    def worldCoordinatesToGuiCoordinates(self, coordinates):
        pass

    def sendTaskToRobot(self):
        print("Assigning task to robot")
        # if else chain depending on task or dictionary of task code
        # linking to function
        pass

    def alertSupervisorRobotIsDone(self):
        print("Robot has finished task. Going to IDLE")
        #find robot in dicionary
        #change robot task to IDLE
        # publish to /supervisor/robots_finished_task that the robot is done
        pass

    # create new robot and add it to the dictionary
    # The message that triggers this callback function should give all the unique information about the robot
    # It should have the unique move base and amcl topics
    def createNewRobot(self, data):
        print("Registering new robot. Robot name: " + str(data.data))
        newRobot = Robot()
        newRobot.name = data.data
        newRobot.status = "OK"
        newRobot.currentTask = "IDLE"
        # if the robot is not already in the dictionary
        if(newRobot.name not in self.activeRobotDictionary.keys()):
            self.activeRobotDictionary[newRobot.name] = newRobot

    # call the methods below in whatever loop this node uses
    # TODO: Nick, depending on the name of the robot, add a list of robots on the left side of the GUI
    def publishRobotStates(self):
        # ask each robot to publish in a list comprehension
        [self.statePublisherForOperator.publish(robot) for robot in self.activeRobotDictionary.values()]


    def publishRobotsLeftInQueue(self):
        self.robotsLeftInQueue.publish()

middleman = Middleman()
while not rospy.is_shutdown():
    middleman.publishRobotStates()
    middleman.publishRobotsLeftInQueue()
    middleman.rate.sleep()