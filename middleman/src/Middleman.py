#!/usr/bin/env python
import time

import rospy
from std_msgs.msg import *
from middleman.msg import Robot, RobotArr, TaskMsg, TaskMsgArr
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from middleman.srv import TaskString, TaskStringResponse
from Task import *




# TODO: Use the name of the robot to determine all the topics for that specific robot since they are all the same
# except the name

# TODO: We're proabbly going to have to have a topic for each of the robot joints as well.... Robot is going
# to encapsulate a lot of information


class Middleman():
    def __init__(self):
        rospy.init_node("middleman", anonymous=True)
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
        self.taskPrios = {
            'NAV': 25,
            'CLN': 50,
            'HLP': 100,
            'DLV': 200,
            'IDLE':0
        }
        self.taskFns = {
            'NAV': self.navTask,
            'CLN': self.clnTask,
            'HLP': self.hlpTask,
            'DLV': self.dlvTask,
            'IDLE': self.idleTask
        }

        self.reassignmentCounter = time.time()
        # list of active tasks happening RN
        self.activeTaskList = []
        # list of tasks that are yet to be assigned
        self.taskPriorityQueue = []
        # dictionary of all active robots
        self.activeRobotDictionary = {}
        # dictionary of all active robot amcl topics
        self.activeRobotAMCLTopics = {}

        # operator queue
        self.robotsForOperator = []
        self.operatorIsBusy = False

        self.rate = rospy.Rate(5)

        # Task Strings: 'task_name robot_name X Y [vars ...]'
        # DLV vars = fromX fromY
        rospy.Subscriber("/supervisor/task", String, self.processTask)
        rospy.Subscriber("/operator/done_helping", String, self.advanceRobotHelpQueue)
        rospy.Subscriber("/operator/request_extra_views_from_robot", String, self.sendAnotherRobotForCameraViews)
        rospy.Subscriber("/robot/stuck", String, self.passRobotToQueueForOperator)
        rospy.Subscriber("/robot/done_task", String, self.alertSupervisorRobotIsDone)
        rospy.Subscriber("/robot/new_robot_running", String, self.createNewRobot)
        rospy.sleep(1)

        # publishers
        self.sendNewRobotToOperator = rospy.Publisher('/operator/new_robot', Robot, queue_size=10)
        self.robotIsAvailableForExtraViews = rospy.Publisher('/operator/robot_is_available_for_extra_views', Bool,
                                                             queue_size=10)
        self.robotsLeftInQueue = rospy.Publisher('/operator/robots_left_in_queue', Int32, queue_size=10)
        # self.robotFinishTask = rospy.Publisher('/supervisor/robots_finished_task', String, queue_size=10)
        self.statePublisherForOperator = rospy.Publisher('/operator/robotState', RobotArr, queue_size=10)
        self.statePublisherForSupervisor = rospy.Publisher('/supervisor/robotState', RobotArr, queue_size=10)
        self.taskListPublisher = rospy.Publisher('/supervisor/taskList', TaskMsgArr, queue_size=10)
        self.taskReassignmentPublisher = rospy.Publisher('/supervisor/taskReassignment', TaskMsgArr, queue_size = 10)
        rospy.sleep(1)

        # servers
        self.taskCodeServer = rospy.Service('/supervisor/taskCodes', TaskString, self.sendTaskCodesToSupervisor)

    #check data length >= 4
    # process task determines which queue to put the task in
    def processTask(self, data):
        # Have to know if the task has been unnassigned
        # have to know what the task is
        # Task Strings: 'task_name robot_name X Y [vars ...]'
        # DLV vars = fromX fromY
        dataList = data.data.split()
        taskName = dataList[0]
        # passed as unassigned if task is not assigned
        robotName = dataList[1]
        print('Split Task Data')

        if len(dataList) > 4:
            variables = dataList[4:]
        else:
            variables = None
        if(taskName != 'SOS'):
            X = float(dataList[2])
            Y = float(dataList[3])
            newTask = Task(taskName, self.taskPrios[taskName], robotName, X, Y, variables)
            newTaskMsg = newTask.convertTaskToTaskMsg()
            if robotName != "unassigned":
                self.taskFns[taskName](newTaskMsg)
                self.activeTaskList.append(newTask)
            else:
                self.taskPriorityQueue.append(newTask)
                self.taskPriorityQueue.sort(key=lambda task: task.getPriority())
            print('Called Task Function')
        elif(taskName == 'SOS'):
            self.passRobotToQueueForOperator(robotName)

    def navTask(self, taskMsg):
        print("Processing Nav Goal")
        # parses Robot name XY string and sends to robots movebase
        currentRobot = self.activeRobotDictionary[taskMsg.robotName]
        currentRobot.currentTask = taskMsg
        currentRobot.currentTaskName = taskMsg.taskName
        self.sendRobotToPos(currentRobot, float(taskMsg.X), float(taskMsg.Y))
        pass

    def clnTask(self, taskMsg):
        print("Processing Cleaning Task")
        # parses Robot name XY string and sends to robots movebase
        currentRobot = self.activeRobotDictionary[taskMsg.robotName]
        currentRobot.currentTask = taskMsg
        currentRobot.currentTaskName = taskMsg.taskName
        self.sendRobotToPos(currentRobot, float(taskMsg.X), float(taskMsg.Y))
        pass

    #@TODO without autonomy work the same as nav task
    def dlvTask(self, taskMsg):
        print("Processing Delivery Task")
        # parses Robot name XY string and sends to robots movebase
        currentRobot = self.activeRobotDictionary[taskMsg.robotName]
        currentRobot.currentTask = taskMsg
        currentRobot.currentTaskName = taskMsg.taskName
        self.sendRobotToPos(currentRobot, float(taskMsg.X), float(taskMsg.Y))

        # data = task.variables.split()
        # toX = data[0]
        # toY = data[1]
        # currentRobot = self.activeRobotDictionary[task.robotName]
        # currentRobot.currentTask = task
        # currentRobot.currentTaskName = task.taskName
        # fromX = task.X
        # fromY = task.Y
        # self.sendRobotToPos(currentRobot, float(fromX), float(fromY))
        pass

    def hlpTask(self, taskMsg):
        print("Processing Help Task")
        # parses Robot name XY string and sends to robots movebase
        currentRobot = self.activeRobotDictionary[taskMsg.robotName]
        currentRobot.currentTask = taskMsg
        currentRobot.currentTaskName = taskMsg.taskName
        self.sendRobotToPos(currentRobot, float(taskMsg.X), float(taskMsg.Y))

    #do nothing
    def idleTask(self, taskMsg):
        currentRobot = self.activeRobotDictionary[taskMsg.robotName]
        currentRobot.currentTask = taskMsg
        currentRobot.currentTaskName = taskMsg.taskName
        self.sendRobotToPos(currentRobot, float(currentRobot.pose.pose.pose.position.x), float(currentRobot.pose.pose.pose.position.y))

    def sendRobotToPos(self, currentRobot, X, Y):
        # publish coordinates to move_base of specific robot
        poseStamped = PoseStamped()
        poseStamped.pose.position.x = X
        poseStamped.pose.position.y = Y
        poseStamped.header.frame_id = 'map'
        # arbitrary orientation for nav goal because operator/automation will take over
        poseStamped.pose.orientation.w = 1
        poseStamped.pose.orientation.z = .16
        topic = ''
        if currentRobot.name == 'trina2':
            topic = '/move_base_simple/goal'
        else:
            topic = '/' + currentRobot.name+'/move_base_simple/goal'
        print(topic)
        self.goal_publisher = rospy.Publisher(topic, PoseStamped, queue_size=10)
        rospy.sleep(2)
        self.goal_publisher.publish(poseStamped)
        print('Publishing Nav Goal')

    # When sos message is published
    # Add to queue, sort
    def passRobotToQueueForOperator(self, robotName):
        print("Passing robot: ", robotName, " to operator")
        # uses name to get robot object
        robotThatNeedsHelp = self.activeRobotDictionary[robotName]
        # change status to OPC
        # task code doesn't change so that the operator knows whats
        # SOS needs to change because the GUI checks currentTaskName for color and other information
        # the task the robot was doing doesn't change in the taskMsg so the operator knows what the robot was doing.
        robotThatNeedsHelp.currentTaskName = "SOS"
        robotThatNeedsHelp.status = "OPC"
        self.robotsForOperator.append(robotThatNeedsHelp)
        self.robotsForOperator.sort(key=lambda robot: robot.currentTask.taskPriority)

        # - check this boolean
        #   if busy - pass
        #   if idle - pop highest priority if list has content
        if not self.operatorIsBusy:
            robotToHelp = self.robotsForOperator.pop()
            self.sendNewRobotToOperator.publish(robotToHelp)
            self.operatorIsBusy = True

    # Operator wants next robot, if available
    def advanceRobotHelpQueue(self, data):
        self.operatorIsBusy = False

        # parse robot name
        robotName = data.data.split()[0]
        robotThatWasHelped = self.activeRobotDictionary[robotName]
        # This gets published, no need to update supervisor
        robotThatWasHelped.status = "OK"
        idleTask = Task("IDLE", self.taskPrios["IDLE"], robotThatWasHelped.name, 0, 0, None)
        idleTaskMsg = idleTask.convertTaskToTaskMsg()
        robotThatWasHelped.currentTask = idleTaskMsg
        robotThatWasHelped.currentTaskName = idleTaskMsg.taskName

        if len(self.robotsForOperator) > 0:
            robotToHelp = self.robotsForOperator.pop()
            print("Sending new robot: ", robotToHelp.name, " ,to operator.")
            self.sendNewRobotToOperator.publish(robotToHelp)
            self.operatorIsBusy = True

    #TODO: add help task to the priority queue or active queue
    def sendAnotherRobotForCameraViews(self, data):
        print("Requesting another robot for camera views")
        robotAvailableForHelp = False
        robotThatWillHelp = None
        # Is there an IDLE robot that CAN be used?
        for robot in self.activeRobotDictionary.values():
            if (robot.currentTask == "IDLE"):
                # find it and grab it from the dictionary
                robot.currentTask = "HLP"
                # publish that a robot is on its way
                robot.status = "OPC"
                robotAvailableForHelp = True
                robotThatWillHelp = robot
                break
        if (not robotAvailableForHelp):
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

    def sendTaskCodesToSupervisor(self, req):
        # colors are IDL-grey Nav-green DLV-orange Help-red CLN-blue SOS-red
        taskCodeStringList = ['IDLE Idle False #9BA8AB', 'NAV Navigation True #75D858', 'DLV Delivery True #B27026', 'HLP Help True #A600FF',
                              'CLN Clean True #00A2FF', 'SOS Stuck False #FF0000']
        return TaskStringResponse(taskCodeStringList)

    def alertSupervisorRobotIsDone(self):
        print("Robot has finished task. Going to IDLE")
        # find robot in dicionary
        # change robot task to IDLE
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

        initialTask = Task("IDLE", self.taskPrios["IDLE"], newRobot.name, 0, 0, None)
        initialTaskMsg = initialTask.convertTaskToTaskMsg()
        newRobot.currentTaskName = initialTaskMsg.taskName
        newRobot.currentTask = initialTaskMsg

        # newRobot.name+
        # print(newRobot.name+'/amcl_pose')
        if newRobot.name == 'trina2':
            self.activeRobotAMCLTopics[
                newRobot.name] = '/amcl_pose'
        else:
            self.activeRobotAMCLTopics[
            newRobot.name] = newRobot.name+'/amcl_pose'
        # if the robot is not already in the dictionary
        if (newRobot.name not in self.activeRobotDictionary.keys()):
            self.activeRobotDictionary[newRobot.name] = newRobot

    # every robot pose updates here
    # data is PoseWithCovarianceStamped
    # def updateRobotPose(self, data):
    # frame_id = data.header.frame_id
    # print(len(frame_id))
    # if len(frame_id) == 5:
    #     robot_to_update = self.activeRobotDictionary['trina2']
    # else:
    #     #this relies on the frame_id having a prepended namespace to identify which robot to update
    #     #dunno if this will happen or not e.g. '/trina2_1/odom'
    #     robot_name = frame_id[1:-5]  #removes the / from beginning and /odom from the end
    #     robot_to_update = self.activeRobotDictionary[robot_name]
    # robot_to_update.pose = data

    # call the methods below in whatever loop this node uses
    # TODO: Nick, depending on the name of the robot, add a list of robots on the left side of the GUI
    def publishRobotStates(self):
        # ask each robot to publish
        robotList = RobotArr()
        for robot in self.activeRobotDictionary.values():
            try:
                pose_msg = rospy.wait_for_message(self.activeRobotAMCLTopics[robot.name], PoseWithCovarianceStamped,
                                                  .05)
                robot.pose = pose_msg
            except:
                pass
            robotList.robots.append(robot)

        self.statePublisherForOperator.publish(robotList)
        self.statePublisherForSupervisor.publish(robotList)

    def assignIdleRobots(self):
        for robot in self.activeRobotDictionary.values():
            # check for IDLE robots
            if robot.currentTaskName == "IDLE" and len(self.taskPriorityQueue) > 0:
                # assign robot a task in th priority queue
                highestPriorityTask = self.taskPriorityQueue.pop()
                highestPriorityTask.robotName = robot.name
                taskMsg = highestPriorityTask.convertTaskToTaskMsg()
                robot.currentTask = taskMsg
                robot.currentTaskName = taskMsg.taskName
                self.taskFns[highestPriorityTask.taskName](taskMsg)
                self.activeTaskList.append(highestPriorityTask)

    def publishRobotsLeftInQueue(self):
        self.robotsLeftInQueue.publish(len(self.robotsForOperator))

    def publishTaskList(self):
        taskMsgList = TaskMsgArr()
        self.taskPriorityQueue.sort(key=lambda t: t.getPriority())
        for task in (self.activeTaskList + self.taskPriorityQueue):
            # turn object into Msg type to publish
            taskMsg = task.convertTaskToTaskMsg()
            taskMsgList.taskMsgs.append(taskMsg)
        self.taskListPublisher.publish(taskMsgList)

    # When a given amount of time has passed, check for reassignment of robot to highest priority task in the priority
    # queue
    def dynamicReassignmentCheck(self):
        fiveMinutes = 60
        if time.time() - self.reassignmentCounter > fiveMinutes and len(self.taskPriorityQueue) > 0:
            print("Time's Up")
            if(len(self.taskPriorityQueue) > 0 and len(self.activeTaskList) > 0):
                highestPriorityTask = self.taskPriorityQueue[-1]
                self.activeTaskList.sort(key=lambda task: task.getPriority())
                lowestPriorityTask = self.activeTaskList[0]
                if highestPriorityTask.getPriority() > lowestPriorityTask.getPriority():
                    print("Reassigning Task " + str(highestPriorityTask.taskName) + ": " + str(highestPriorityTask.getPriority()))
                    swappedTasks = TaskMsgArr()
                    # swap the active task with high  [Active, High Priority]
                    swappedTasks.taskMsgs.append(lowestPriorityTask.convertTaskToTaskMsg())
                    swappedTasks.taskMsgs.append(highestPriorityTask.convertTaskToTaskMsg())
                    self.taskReassignmentPublisher.publish(swappedTasks)
                    #remove from priority queue
                    self.taskPriorityQueue.pop()
                    #reassign robot name
                    highestPriorityTask.robotName = lowestPriorityTask.robotName
                    #add new task to active
                    self.activeTaskList.append(highestPriorityTask)
                    #remove now inactive from activeList
                    self.activeTaskList.remove(lowestPriorityTask)
                    robotBeingReassigned = self.activeRobotDictionary[lowestPriorityTask.robotName]
                    robotBeingReassigned.currentTask = highestPriorityTask.convertTaskToTaskMsg()
                    robotBeingReassigned.currentTaskName = highestPriorityTask.taskName
                    #reassign active task to unassigned status
                    lowestPriorityTask.robotName = 'unassigned'
                    #add previously active task to priority queue
                    self.taskPriorityQueue.append(lowestPriorityTask)
                    self.reassignmentCounter = time.time()
            self.reassignmentCounter = time.time()



middleman = Middleman()
while not rospy.is_shutdown():
    middleman.assignIdleRobots()
    middleman.publishRobotStates()
    middleman.publishRobotsLeftInQueue()
    middleman.publishTaskList()
    middleman.dynamicReassignmentCheck()
    middleman.rate.sleep()
