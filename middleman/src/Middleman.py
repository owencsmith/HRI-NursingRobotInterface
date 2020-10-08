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
        #subscribers (should actually be Service/Client)
        rospy.Subscriber("/supervisor/nav_task", String, self.processNavTask)
        rospy.Subscriber("/supervisor/sos", String, self.passRobotToQueueForOperator)
        rospy.Subscriber("/operator/done_helping", String, self.changeRobotToIDLE)

        #publishers
        # TODO: Make our own Robot message with fields of robot object
        # TODO: when operator is done with a robot, pop from list and send new one using Service/Client (not publisher)
        sendNewRobotToOperator = rospy.Publisher('/operator/new_robot', String, queue_size=10)


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

    def changeRobotToIDLE(self,data):
        #parse robot name
        robotName = ""
        robotThatWasHelped = self.activeRobotDictionary[robotName]
        #This gets published, no need to update supervisor
        robotThatWasHelped.status = "OK"
        robotThatWasHelped.currentTask = "IDL"
        pass

    # call in whatever loop this node uses
    # TODO: Nick, depending on the name of the robot, add a list of robots on the left side of the GUI
    def publishRobotStates(self):
        # ask each robot to publish in a list comprehension
        [robot.publish() for robot in self.activeRobotDictionary.values()]

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

