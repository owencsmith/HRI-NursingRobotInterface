#!/usr/bin/env python
import rospy
class Middleman():
    def __init__(self):
        rospy.init_node("middleman", anonymous = True)
        # dictionary for task codes
        self.taskCodes = {}

        # dictionary of all active robots
        self.activeRobotDictionary = {}

        #operator queue
        self.robotsForOperator = []

        #TODO:
        rospy.Subscriber("/supervisor/nav_task", String, self.processNavTask)
        rospy.Subscriber("/supervisor/sos", String, self.passRobotToQueueForOperator)
        rospy.Subscriber("/operator/done_helping", String, self.advanceRobotHelpQueue)
        rospy.Subscriber("/operator/request_extra_views_from_robot", String, self.sendAnotherRobotForCameraViews)
        rospy.Subscriber("/robot/stuck", String, self.passRobotToQueueForOperator)
        rospy.Subscriber("/robot/done_task", String, self.alertSupervisorRobotIsDone)

        #publishers
        # TODO: Make our own Robot message with fields of robot object
        # TODO: when operator is done with a robot, pop from list and send new one using Service/Client (not publisher)
        self.sendNewRobotToOperator  = rospy.Publisher('/operator/new_robot', String, queue_size=10)
        self.robotIsAvailableForExtraViews = rospy.Publisher('/operator/robot_is_available_for_extra_views', Bool, queue_size = 10)
        self.robotsLeftInQueue = rospy.Publisher('/operator/robots_left_in_queue', Int32, queue_size = 10)
        self.robotsLeftInQueue = rospy.Publisher('/supervisor/robots_finished_task', String, queue_size=10)
        pass


    def processNavTask(self, data):
        # parses Robot name XY string and sends to robots movebase
        robotName = ""
        X = ""
        Y = ""
        currentRobot = self.activeRobotDictionary[robotName]
        currentRobot.currentTask = "NAV"
        worldCoordinates = self.guiCoordinatesToWorldCoordinates([X,Y])
        #publish coordinates to move_base of specific robot
        currentRobot.publish()
        pass

    def passRobotToQueueForOperator(self, data):
        #get robot name
        robotName = data.data
        #uses name to get robot object
        robotThatNeedsHelp = self.activeRobotDictionary[robotName]
        #change status to OPC
        #task code doesn't change so that the operator knows whats up
        robotThatNeedsHelp.status = "OPC"
        #add the robot to the operator queue
        self.robotsForOperator.append(robotThatNeedsHelp)

        pass

    def advanceRobotHelpQueue(self,data):
        #parse robot name
        robotName = ""
        robotThatWasHelped = self.activeRobotDictionary[robotName]
        #This gets published, no need to update supervisor
        robotThatWasHelped.status = "OK"
        robotThatWasHelped.currentTask = "IDL"
        # pop another robot from the queue if queue is not empty
        # publish that robot to /operator/new_robot
        pass

    def sendAnotherRobotForCameraViews(self):
        # Is there an IDLE robot that CAN be used?
            # find it and grab it from the dictionary
            # publish that a robot is on its way
        # If not
            # publish that no robot could be used right now

        # parse the string for the robot to navigate to for more views
        # grab the pose of the robot to navigate to
        # Tell the IDLE robot to navigate to the robot that needs more views
        # set the task of the IDLE robot to NAV
        # set the status of the IDLE robot to OPC
        pass

    def guiCoordinatesToWorldCoordinates(self, coordinates):
        pass

    def worldCoordinatesToGuiCoordinates(self, coordinates):
        pass

    def publishAllRobotStates(self):
        pass

    def sendTaskToRobot(self):
        # if else chain depending on task or dictionary of task code
        # linking to function
        pass

    def alertSupervisorRobotIsDone(self):
        #find robot in dicionary
        #change robot task to IDLE
        # publish to /supervisor/robots_finished_task that the robot is done
        pass
    # call the methods below in whatever loop this node uses
    # TODO: Nick, depending on the name of the robot, add a list of robots on the left side of the GUI
    def publishRobotStates(self):
        # ask each robot to publish in a list comprehension
        [robot.publish() for robot in self.activeRobotDictionary.values()]

    def publishRobotsLeftInQueue(self):
        self.robotsLeftInQueue.publish()



