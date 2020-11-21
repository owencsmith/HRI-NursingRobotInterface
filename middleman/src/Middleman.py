#!/usr/bin/env python
import time

import rospy
from std_msgs.msg import *
from middleman.msg import Robot, RobotArr, TaskMsg, TaskMsgArr
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from middleman.srv import TaskString, TaskStringResponse
from Task import *
import numpy as np

# Todo: Change initialization of supervisor and operator publisher & subscriber

# Todo: Allocate robots to supervisor based on num robots and num supervisor

# Todo: Allocate robots to operators (see which ones are busy) -maybe add to hlp task variables
#  -service client for busy

# Todo: What happens if a new supervisor becomes active with already functioning robot team? Do we steal robots from
#  other supervisor?

#TODO: How to do Viapoints
    # A list of nav tasks
    # store it in variables for nav
         # OR send a list of nav tasks
    #

# Todo: Should there be a minimum number of robots that a supervisor needs to actually 'log in'? e.g. 2 robots in
#  fleet, 2 supervisors doesnt make sense

# Todo: Need a priority queue, active task list for every supervisor - maybe we do move everything into a supervisor
#  object. Still dont pass the middleman into it if possible

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
            'IDLE': 0
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
        rospy.Subscriber("/operator/release_help_robot", String, self.releaseFromHelp)
        rospy.Subscriber("/supervisor/removeTask", String, self.removeFromPriorityQueue)
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
        self.taskReassignmentPublisher = rospy.Publisher('/supervisor/taskReassignment', TaskMsgArr, queue_size=10)
        self.TellOperatorThatRobotCameraIsAvailable = rospy.Publisher('/operator/robotForExtraCamera', String, queue_size=10)
        rospy.sleep(1)

        # servers
        self.taskCodeServer = rospy.Service('/supervisor/taskCodes', TaskString, self.sendTaskCodesToSupervisor)

    # check data length >= 4
    # process task determines which queue to put the task in
    def processTask(self, data):
        """
        This function determines which task is being asked to run on the robot
        and calls the apprropriate task function. It also adds the task to the
        active task list if a robot is assigned, or a priority queue if no robot
        is assigned. Please look at the task code data structure above to see what that task codes mean. 
        :param data: the string containing the robot and task information.
                     i.e. <task name> <robot name> <X coordinate> <Y coordinate>
        :return None
        """

        # Have to know if the task has been unnassigned
        # have to know what the task is
        # Task Strings: 'task_name robot_name X Y [vars ...]'
        # DLV vars = fromX fromY
        if type(data) == str:
            dataList = data.split()
        else:
            dataList = data.data.split()
        taskName = dataList[0]
        # passed as unassigned if task is not assigned
        robotName = dataList[1]

        yaw = float(dataList[4])
        raisePriority = dataList[5]
        if(raisePriority == "False"):
            priorityRaised = False
        else:
            priorityRaised = True
        print('Split Task Data')
        print("PRIORITY RAISED: ", priorityRaised)
        print(len(dataList))
        print(dataList)

        # if len(dataList) > 5:
        #     variables = dataList[5]
        # else:
        #     variables = " "

        if (taskName != 'SOS'):
            X = float(dataList[2])
            Y = float(dataList[3])
            newTask = Task(taskName, self.taskPrios[taskName], robotName, X, Y, yaw, priorityRaised)
            newTaskMsg = newTask.convertTaskToTaskMsg()
            if robotName != "unassigned":
                # remove current task form active task list
                oldTaskMsg = self.activeRobotDictionary[robotName].currentTask
                for task in self.activeTaskList:
                    if oldTaskMsg.ID == task.getID():
                        self.activeTaskList.remove(task)
                        break
                self.taskFns[taskName](newTaskMsg)
                self.activeTaskList.append(newTask)
            else:
                self.taskPriorityQueue.append(newTask)
                self.taskPriorityQueue.sort(key=lambda task: task.getPriority())
            print('Called Task Function')
        elif (taskName == 'SOS'):
            self.passRobotToQueueForOperator(robotName)

    def navTask(self, taskMsg):
        """
        Performs a navigation task. Sets the robots task to NAV and commands it to a specified
        position.
        :param taskMsg: A taskMsg ROS message. Contains X and Y information as well as task code.
        :return:  none
        """
        print("Processing Nav Goal")
        # parses Robot name XY string and sends to robots movebase
        currentRobot = self.activeRobotDictionary[taskMsg.robotName]
        currentRobot.currentTask = taskMsg
        currentRobot.currentTaskName = taskMsg.taskName
        self.sendRobotToPos(currentRobot, float(taskMsg.X), float(taskMsg.Y), float(taskMsg.yaw))
        pass

    def clnTask(self, taskMsg):
        """
         Sets the robots task to CLN and commands it to a specified location. Since no robot autonomy has yet been
         implemented, this function can only navigate to the goal. Future work can implement the autonomy.
         :param taskMsg: A taskMsg ROS message. Contains X and Y information as well as task code.
         :return:  none
         """
        print("Processing Cleaning Task")
        # parses Robot name XY string and sends to robots movebase
        currentRobot = self.activeRobotDictionary[taskMsg.robotName]
        currentRobot.currentTask = taskMsg
        currentRobot.currentTaskName = taskMsg.taskName
        self.sendRobotToPos(currentRobot, float(taskMsg.X), float(taskMsg.Y), float(taskMsg.yaw))
        pass

    # @TODO without autonomy work the same as nav task
    def dlvTask(self, taskMsg):
        """
         Sets the robots task to DLV and commands it to a specified location. Since no robot autonomy has yet been
         implemented, this function can only navigate to the goal. Future work can implement the autonomy.
         :param taskMsg: A taskMsg ROS message. Contains X and Y information as well as task code.
         :return:  none
         """
        print("Processing Delivery Task")
        # parses Robot name XY string and sends to robots movebase
        currentRobot = self.activeRobotDictionary[taskMsg.robotName]
        currentRobot.currentTask = taskMsg
        currentRobot.currentTaskName = taskMsg.taskName
        self.sendRobotToPos(currentRobot, float(taskMsg.X), float(taskMsg.Y), float(taskMsg.yaw))

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
        """
         Sets the robots task to HLP and commands it to a specified location. Since no robot autonomy has yet been
         implemented, this function can only navigate to the goal. Future work can implement the autonomy.
         :param taskMsg: A taskMsg ROS message. Contains X and Y information as well as task code.
         :return:  none
         """
        print("Processing Help Task")
        # parses Robot name XY string and sends to robots movebase
        currentRobot = self.activeRobotDictionary[taskMsg.robotName]
        currentRobot.status = 'OPC'
        currentRobot.currentTask = taskMsg
        currentRobot.currentTaskName = taskMsg.taskName
        print(taskMsg.variables)
        self.sendRobotToPos(currentRobot, float(taskMsg.X), float(taskMsg.Y), float(taskMsg.yaw))
        self.TellOperatorThatRobotCameraIsAvailable.publish(taskMsg.robotName)

    # do nothing
    def idleTask(self, taskMsg):
        """
         Sets the robots task to IDLE and commands it to a specified location. The robot will stop and wait for a
         different task.
         :param taskMsg: A taskMsg ROS message.
         :return:  none
         """
        currentRobot = self.activeRobotDictionary[taskMsg.robotName]
        currentRobotOrient = currentRobot.pose.pose.pose.orientation
        quat_list = [currentRobotOrient.x, currentRobotOrient.y, currentRobotOrient.z,
                     currentRobotOrient.w]
        robotEuler = euler_from_quaternion(quat_list)
        robYaw = robotEuler[2]
        self.sendRobotToPos(currentRobot, float(currentRobot.pose.pose.pose.position.x),
                            float(currentRobot.pose.pose.pose.position.y), float(robYaw))

    def sendRobotToPos(self, currentRobot, X, Y, yaw=0):
        """
        Given a robot and a location, sends the robot to that location by publishing to
        'robot_namespace'/move_base_simple/goal
        :param currentRobot: The robot to move
        :param X: the x location on the map
        :param Y: the y location on the map
        """
        # publish coordinates to move_base of specific robot
        poseStamped = PoseStamped()
        poseStamped.pose.position.x = X
        poseStamped.pose.position.y = Y
        poseStamped.header.frame_id = 'map'

        # add 90 deg for offset with UI. Our Z is out of the page, 0 is pointing left
        q_orientation = quaternion_from_euler(0, 0, (-yaw) + np.pi)

        # arbitrary orientation for nav goal because operator/automation will take over
        poseStamped.pose.orientation.x = q_orientation[0]
        poseStamped.pose.orientation.y = q_orientation[1]
        poseStamped.pose.orientation.w = q_orientation[2]
        poseStamped.pose.orientation.z = q_orientation[3]
        topic = ''
        if currentRobot.name == 'trina2':
            topic = '/move_base_simple/goal'
        else:
            topic = '/' + currentRobot.name + '/move_base_simple/goal'
        print(topic)
        self.goal_publisher = rospy.Publisher(topic, PoseStamped, queue_size=10)
        rospy.sleep(2)
        self.goal_publisher.publish(poseStamped)
        print('Publishing Nav Goal')

    def passRobotToQueueForOperator(self, robotName):
        """
        This is a callback for the SOS message from the supervisor. Adds a robot in trouble to the operator queue and
        sorts them by priority.
        :param robotName: the robot that needs help
        """
        if robotName == 'unassigned':
            return
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

    def advanceRobotHelpQueue(self, data):
        """
        When an operator finishes with a robot, if there is another robot in need of assistance load it into the
        operator GUI
        :param data: the robot name that was previously being helped by the operator
        """
        self.operatorIsBusy = False

        # parse robot name
        robotName = data.data
        robotThatWasHelped = self.activeRobotDictionary[robotName]
        # This gets published, no need to update supervisor
        robotThatWasHelped.status = "OK"
        idleTask = Task("IDLE", self.taskPrios["IDLE"], robotThatWasHelped.name, 0, 0, False, " ")
        idleTaskMsg = idleTask.convertTaskToTaskMsg()
        robotThatWasHelped.currentTask = idleTaskMsg
        robotThatWasHelped.currentTaskName = idleTaskMsg.taskName
        self.idleTask(idleTaskMsg)

        # Find the robot that was helping (if any), assign it a new idle task
        for robot in self.activeRobotDictionary.values():
            # check for IDLE robots
            if robot.currentTaskName == "HLP":
                info_str = 'IDLE' + ' ' + robot.name + ' ' + '0' + ' ' + '0' + ' ' + '0' + ' ' + 'False' + ' '
                self.processTask(info_str)
                break

        if len(self.robotsForOperator) > 0:
            robotToHelp = self.robotsForOperator.pop()
            print("Sending new robot: ", robotToHelp.name, " ,to operator.")
            self.sendNewRobotToOperator.publish(robotToHelp)
            self.operatorIsBusy = True

    def releaseFromHelp(self, data):
        # Find the robot that was helping (if any), assign it a new idle task
        for robot in self.activeRobotDictionary.values():
            # check for IDLE robots
            if robot.currentTaskName == "HLP":
                info_str = 'IDLE' + ' ' + robot.name + ' ' + '0' + ' ' + '0' + ' ' + '0' + ' ' + 'False' + ' '
                self.processTask(info_str)
                break

    def removeFromPriorityQueue(self, data):
        ID = data.data
        print("ID: ", data.data)
        # remove from the task priority list the task with this ID
        for i, task in enumerate(self.taskPriorityQueue):
            if(task.getID() == ID):
                self.taskPriorityQueue.remove(task)

    # TODO: add help task to the priority queue or active queue
    def sendAnotherRobotForCameraViews(self, data):
        """
        Part of the help task. Called when the operator requests another camera view (provided by head camera of
        additional robot
        :param data: robot name of the robot that is being helped by the operator
        """
        print("Requesting another robot for camera views")
        robotAvailableForHelp = False
        robotThatWillHelp = None
        # Is there an IDLE robot that CAN be used?
        robotToNavigateTo = self.activeRobotDictionary[data.data.split()[0]]
        # grab the pose of the robot to navigate to
        robotToNavigateToPose = robotToNavigateTo.pose.pose.pose.position
        robotToNavigateToOrient = robotToNavigateTo.pose.pose.pose.orientation
        quat_list = [robotToNavigateToOrient.x, robotToNavigateToOrient.y, robotToNavigateToOrient.z, robotToNavigateToOrient.w]
        robotEuler = euler_from_quaternion(quat_list)
        robYaw = robotEuler[2]
        xBuffer = np.sin(robYaw)*1
        yBuffer = np.cos(robYaw)*1
        #print(robotEuler)
        robotToNavigateToX = robotToNavigateToPose.x - xBuffer
        robotToNavigateToY = robotToNavigateToPose.y - yBuffer

        # todo, try and fix this now....
        info_str = 'HLP' + ' ' + 'unassigned ' + str(robotToNavigateToX) + ' ' + str(robotToNavigateToY) + ' ' + str(robYaw+(5*np.pi/4)) + ' ' + 'False' + ' '

        print(info_str)
        self.processTask(info_str)
        # # parse the string for the robot to navigate to for more views
        # helpTask = Task("HLP", self.taskPrios["HLP"], robotThatWillHelp.name, robotToNavigateToX, robotToNavigateToY, None)
        # helpTaskMsg = helpTask.convertTaskToTaskMsg()
        # self.hlpTask(helpTaskMsg)


        #
        # if(not robotAvailableForHelp):
        #     helpTask = Task("HLP", self.taskPrios["HLP"], "unassigned", robotToNavigateToX, robotToNavigateToY, None)
        #     self.taskPriorityQueue.append(helpTask)


    def sendTaskCodesToSupervisor(self, req):
        """
        A boot message that provides color display infor and task type to the supervisor UI
        :param req:
        :return:
        """
        # colors are IDL-grey Nav-green DLV-orange Help-red CLN-blue SOS-red
        taskCodeStringList = ['IDLE Idle False #9BA8AB True', 'NAV Navigation True #75D858 True',
                              'DLV Delivery True #B27026 True', 'HLP Help True #A600FF False',
                              'CLN Clean True #00A2FF True', 'SOS Stuck False #FF0000 True']
        return TaskStringResponse(taskCodeStringList)

    # TODO: What does this do??. Nothing cause we have no way of knowing robot is done. (Autonomy not implemented)
    def alertSupervisorRobotIsDone(self):
        """
        THIS DOES NOTHING I THINK
        :return:
        """
        print("Robot has finished task. Going to IDLE")
        # find robot in dicionary
        # change robot task to IDLE
        # publish to /supervisor/robots_finished_task that the robot is done
        pass

    # create new robot and add it to the dictionary
    # The message that triggers this callback function should give all the unique information about the robot
    # It should have the unique move base and amcl topics
    def createNewRobot(self, data):
        """
        When a robot sends a boot message. Builds a robot message type and adds it to the robots tracked by the
         middleman
        :param data: the robot name
        :return:
        """
        print("Registering new robot. Robot name: " + str(data.data))
        newRobot = Robot()
        newRobot.name = data.data
        newRobot.status = "OK"

        initialTask = Task("IDLE", self.taskPrios["IDLE"], newRobot.name, 0, 0, 0, False, " ")
        initialTaskMsg = initialTask.convertTaskToTaskMsg()
        newRobot.currentTaskName = initialTaskMsg.taskName
        newRobot.currentTask = initialTaskMsg
        self.activeTaskList.append(initialTask)

        # newRobot.name+
        # print(newRobot.name+'/amcl_pose')
        if newRobot.name == 'trina2':
            self.activeRobotAMCLTopics[
                newRobot.name] = '/amcl_pose'
        else:
            self.activeRobotAMCLTopics[
                newRobot.name] = newRobot.name + '/amcl_pose'
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
    def publishRobotStates(self):
        """
        Publishes the states of the robots in the robot list tracked by the middleman to the supervisor and operator in
        one compact message
        """
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
            #print(type(robot.currentTask.variables))

        self.statePublisherForOperator.publish(robotList)
        self.statePublisherForSupervisor.publish(robotList)

    def assignIdleRobots(self):
        """
        Checks for idle robots if there are tasks in the priority queue. Assigns them the tasks so little to no downtime
        until all tasks are completed.
        """
        for robot in self.activeRobotDictionary.values():
            # check for IDLE robots
            if robot.currentTaskName == "IDLE" and len(self.taskPriorityQueue) > 0:
                # assign robot a task in th priority queue
                highestPriorityTask = self.taskPriorityQueue.pop()
                highestPriorityTask.robotName = robot.name
                taskMsg = highestPriorityTask.convertTaskToTaskMsg()

                #print(self.activeTaskList)
                for task in self.activeTaskList:
                    if robot.currentTask.ID == task.getID():
                        self.activeTaskList.remove(task)
                        break
                #print(self.activeTaskList)

                robot.currentTask = taskMsg
                robot.currentTaskName = taskMsg.taskName
                self.taskFns[highestPriorityTask.taskName](taskMsg)
                self.activeTaskList.append(highestPriorityTask)

    def publishRobotsLeftInQueue(self):
        """
        Publishes the length of the robot operator help queue
        """
        self.robotsLeftInQueue.publish(len(self.robotsForOperator))

    def publishTaskList(self):
        """
        Publishes the task list (active tasks being completed and unassigned priority sorted tasks)
        """
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
        """
        When a given amount of time has passed, check for reassignment of robot to highest priority task in the priority
        queue
        """
        fiveMinutes = 60
        # if the time has elapsed for a reassinment check, and the length of both queues are greater than zero (meaning we can swap)
        if((time.time() - self.reassignmentCounter > fiveMinutes) and (len(self.taskPriorityQueue) > 0) and (len(self.activeTaskList) > 0)):
                highestPriorityTask = self.taskPriorityQueue[-1]
                self.activeTaskList.sort(key=lambda task: task.getPriority())
                lowestPriorityTask = self.activeTaskList[0]
                if(lowestPriorityTask.taskName != "IDLE"):
                    if highestPriorityTask.getPriority() > lowestPriorityTask.getPriority():
                        print("Reassigning Task " + str(highestPriorityTask.taskName) + ": " + str(
                            highestPriorityTask.getPriority()))
                        swappedTasks = TaskMsgArr()
                        # swap the active task with high  [Active, High Priority]
                        swappedTasks.taskMsgs.append(lowestPriorityTask.convertTaskToTaskMsg())
                        swappedTasks.taskMsgs.append(highestPriorityTask.convertTaskToTaskMsg())
                        self.taskReassignmentPublisher.publish(swappedTasks)
                        # remove from priority queue
                        self.taskPriorityQueue.pop()
                        # reassign robot name
                        highestPriorityTask.robotName = lowestPriorityTask.robotName
                        # add new task to active
                        self.activeTaskList.append(highestPriorityTask)
                        self.taskFns[highestPriorityTask.taskName](highestPriorityTask)  # call task function

                        # remove now inactive from activeList
                        self.activeTaskList.remove(lowestPriorityTask)
                        robotBeingReassigned = self.activeRobotDictionary[lowestPriorityTask.robotName]
                        robotBeingReassigned.currentTask = highestPriorityTask.convertTaskToTaskMsg()
                        robotBeingReassigned.currentTaskName = highestPriorityTask.taskName
                        # reassign active task to unassigned status
                        lowestPriorityTask.robotName = 'unassigned'
                        # add previously active task to priority queue
                        self.taskPriorityQueue.append(lowestPriorityTask)

                self.reassignmentCounter = time.time()

    def checkNavStatus(self):
        navTolerance = .1
        for robot in self.activeRobotDictionary.values():
            if(robot.currentTask.taskName == "NAV"):
                if((abs(robot.pose.pose.pose.position.x - robot.currentTask.X) <= navTolerance) and
                        (abs(robot.pose.pose.pose.position.y - robot.currentTask.Y) <= navTolerance)):
                        # if we do via points, instead of IDLE task, send next position in via points list
                        info_str = 'IDLE' + ' ' + robot.name + ' ' + '0' + ' ' + '0' + ' ' + '0' + ' ' + 'False' + ' '
                        self.processTask(info_str)

            pass

middleman = Middleman()
while not rospy.is_shutdown():
    middleman.assignIdleRobots()
    middleman.publishRobotStates()
    middleman.publishRobotsLeftInQueue()
    middleman.publishTaskList()
    middleman.dynamicReassignmentCheck()
    #middleman.checkNavStatus()
    middleman.rate.sleep()
