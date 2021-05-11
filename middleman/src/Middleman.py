#!/usr/bin/env python
import math
import time

import rospy
from std_msgs.msg import *
from nav_msgs.msg import OccupancyGrid
from middleman.msg import Robot, RobotArr, TaskMsg, TaskMsgArr
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from middleman.srv import TaskString, TaskStringResponse
from Task import *
from Operator import *
from Supervisor import *
from searchCoordinator import *
import numpy as np


# Todo: Change initialization of supervisor and operator publisher & subscriber

# Todo: Allocate robots to supervisor based on num robots and num supervisor

# Todo: Allocate robots to operators (see which ones are busy) -maybe add to hlp task variables
#  -service client for busy

# Todo: What happens if a new supervisor becomes active with already functioning robot team? Do we steal robots from
#  other supervisor?

# TODO: How to do Viapoints
# A list of nav tasks
# store it in variables for nav
# OR send a list of nav tasks
#

# Todo: Should there be a minimum number of robots that a supervisor needs to actually 'log in'? e.g. 2 robots in
#  fleet, 2 supervisors doesnt make sense

# Todo: Need a priority queue, active task list for every supervisor - maybe we do move everything into a supervisor
#  object. Still dont pass the middleman into it if possible

class Middleman():

    def __init__(self, search_algorithm):

        self.search_algorithm = search_algorithm

        rospy.init_node("middleman", anonymous=True)
        print("Initializing Middleman Node")

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
            'HLP': 200,
            'DLV': 100,
            'SEARCH': 125,
            'IDLE': 0
        }
        self.taskFns = {
            'NAV': self.navTask,
            'CLN': self.clnTask,
            'HLP': self.hlpTask,
            'DLV': self.dlvTask,
            'IDLE': self.idleTask,
            'SEARCH': self.searchTask
        }

        self.reassignmentCounter = time.time()

        # dictionary of all active robots
        self.activeRobotDictionary = {}
        # dictionaty of all active operators
        self.activeOperatorDictionary = {}
        # dictionaty of all active supervisors
        self.activeSupervisorDictionary = {}

        # dictionary of all active robot amcl topics
        self.activeRobotAMCLTopics = {}

        self.unsupervisedTasks = []

        # operator queue
        self.robotsForOperator = []
        self.operatorIsBusy = False

        self.rate = rospy.Rate(5)

        # Task Strings: 'task_name robot_name X Y [vars ...]'
        # DLV vars = fromX fromY
        rospy.Subscriber("/supervisor/task", String, self.processTask)
        rospy.Subscriber("/supervisor/removeTask", String, self.removeFromPriorityQueue)
        rospy.Subscriber("/operator/done_helping", String, self.advanceRobotHelpQueue)
        rospy.Subscriber("/operator/request_extra_views_from_robot", String, self.sendAnotherRobotForCameraViews)
        rospy.Subscriber("/operator/release_help_robot", String, self.releaseFromHelp)
        rospy.Subscriber("/operator/new_operator_ui", String, self.registerNewOperator)
        rospy.Subscriber("/operator/seppuku", String, self.unregisterOperator)
        rospy.Subscriber("/supervisor/new_supervisor_ui", String, self.registerNewSupervisor)
        rospy.Subscriber("/supervisor/seppuku", String, self.unregisterSupervisor)
        rospy.Subscriber("/robot/stuck", String, self.passRobotToQueueForOperator)
        rospy.Subscriber("/robot/done_task", String, self.alertSupervisorRobotIsDone)
        rospy.Subscriber("/robot/new_robot_running", String, self.createNewRobot)

        # print("A")         # The sleeps hang or some reason
        # rospy.sleep(1)
        # print("A")

        # publishers
        self.statePublisherForSupervisor = rospy.Publisher('/supervisor/robotState', RobotArr, queue_size=10)
        self.robotsLeftInQueue = rospy.Publisher('/operator/robots_left_in_queue', Int32, queue_size=10)
        # self.robotFinishTask = rospy.Publisher('/supervisor/robots_finished_task', String, queue_size=10)
        self.statePublisherForOperator = rospy.Publisher('/operator/robotState', RobotArr, queue_size=10)

        # print("A")
        # rospy.sleep(1)
        # print("A")

        # servers
        self.taskCodeServer = rospy.Service('/supervisor/taskCodes', TaskString, self.sendTaskCodesToSupervisor)
        self.map = OccupancyGrid()
        self.mapSubscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.collaborativeSearchSubscriber = rospy.Subscriber('/collab', String, self.guard_searching)

        c = (self.search_algorithm==1)
        self.sc = SearchCoordinator("HospitalMapCleaned_filledin_black_border_clean2.png", cluster=c)

        self.searchStarted = False
        self.guardDictionary = {}

        self.robot_stuck_count = {}
        self.robot_previous_poses = {}
        self.robot_resend_amt = {}

        print("Initialization Done \n")

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
            if (data == ""):
                return
        else:
            if (data.data == ""):
                return
            dataList = data.data.split()

        supervisorID = dataList[0]
        supervisor = self.activeSupervisorDictionary[supervisorID]
        taskName = dataList[1]
        # passed as unassigned if task is not assigned
        robotName = dataList[2]

        yaw = -float(dataList[5]) + np.pi
        raisePriority = dataList[6]
        if (raisePriority == "False"):
            priorityRaised = False
        else:
            priorityRaised = True
        # print('Split Task Data')
        # print("PRIORITY RAISED: ", priorityRaised)

        if len(dataList) > 7:
            variables = dataList[7]
            print(variables)
        else:
            variables = ""

        if (taskName != 'SOS'):
            X = float(dataList[3])
            Y = float(dataList[4])
            newTask = Task(taskName, self.taskPrios[taskName], robotName, X, Y, yaw, priorityRaised, supervisorID,
                           variables)
            newTaskMsg = newTask.convertTaskToTaskMsg()
            if robotName != "unassigned":
                # remove current task form active task list
                oldTaskMsg = self.activeRobotDictionary[robotName].currentTask
                for task in supervisor.activeTaskList:
                    if oldTaskMsg.ID == task.getID():
                        supervisor.activeTaskList.remove(task)
                        break
                self.taskFns[taskName](newTaskMsg)
                supervisor.activeTaskList.append(newTask)
            else:
                supervisor.taskPriorityQueue.append(newTask)
                supervisor.taskPriorityQueue.sort(key=lambda task: task.getPriority())
            # print('Called Task Function')
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

    def searchTask(self, taskMsg):
        # print("Processing Search Goal")
        # parses Robot name XY string and sends to robots movebase

        splitvar = taskMsg.variables.split(",")

        wh = self.sc.get_width_and_height()
        robots = self.activeRobotDictionary.keys()
        print("MM: creating robot dict for SC")
        robot_dict_for_sc = {}
        for robot in robots:
            print(robot)
            x = self.activeRobotDictionary[robot].pose.pose.pose.position.x
            y = self.activeRobotDictionary[robot].pose.pose.pose.position.y
            robot_pt = self.transform_realworld_to_map((x,y),wh)
            robot_dict_for_sc[robot] = robot_pt
        print(robot_dict_for_sc)

        self.sc.start_search(splitvar, robot_dict=robot_dict_for_sc)
        # self.searchStarted = True
        rospy.loginfo("STARTED SEARCH")
        # self.sendRobotToPos(currentRobot, float(taskMsg.X), float(taskMsg.Y), float(taskMsg.yaw))

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

        # check which operator needs help and doesn't have it?
        for operator in self.activeOperatorDictionary.values():
            if (operator.needHelpingRobot):
                operator.TellOperatorThatRobotCameraIsAvailable.publish(taskMsg.robotName)
                # link the helping robot to the operator
                operator.linkToHelpingRobot(currentRobot)
                break

    # do nothing
    def idleTask(self, taskMsg):
        """
         Sets the robots task to IDLE and commands it to a specified location. The robot will stop and wait for a
         different task.
         :param taskMsg: A taskMsg ROS message.
         :return:  none
         """
        currentRobot = self.activeRobotDictionary[taskMsg.robotName]
        currentRobot.currentTask = taskMsg
        currentRobot.currentTaskName = taskMsg.taskName
        currentRobot.status = "OK"

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
        q_orientation = quaternion_from_euler(0, 0, yaw)

        # arbitrary orientation for nav goal because operator/automation will take over
        poseStamped.pose.orientation.x = q_orientation[0]
        poseStamped.pose.orientation.y = q_orientation[1]
        poseStamped.pose.orientation.w = q_orientation[3]
        poseStamped.pose.orientation.z = q_orientation[2]
        topic = ''
        if currentRobot.name == 'trina2':
            topic = '/move_base_simple/goal'
        else:
            topic = '/' + currentRobot.name + '/move_base_simple/goal'
        # print(topic)
        self.goal_publisher = rospy.Publisher(topic, PoseStamped, queue_size=10)
        rospy.sleep(2)
        self.goal_publisher.publish(poseStamped)
        # print('Publishing Nav Goal')

    def passRobotToQueueForOperator(self, robotName):
        """
        This is a callback for the SOS message from the supervisor. Adds a robot in trouble to the operator queue and
        sorts them by priority.
        :param robotName: the robot that needs help
        """
        if robotName == 'unassigned':
            return
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

        # determine which operator this goes to
        for operator in self.activeOperatorDictionary.values():
            # - check this boolean
            #   if busy - pass
            #   if idle - pop highest priority if list has content
            if not operator.operatorIsBusy:
                robotToHelp = self.robotsForOperator.pop()
                operator.sendNewRobotToOperator.publish(robotToHelp)
                operator.operatorIsBusy = True
                operator.linkToRobot(robotToHelp)
                print("Passing robot: " + robotToHelp.name + " to operator: " + operator.operatorID)
                break

    def advanceRobotHelpQueue(self, data):
        """
        When an operator finishes with a robot, if there is another robot in need of assistance load it into the
        operator GUI
        :param data: the robot name that was previously being helped by the operator
        """

        # parse robot name
        robotName = data.data
        robotThatWasHelped = self.activeRobotDictionary[robotName]
        # This gets published, no need to update supervisor
        robotThatWasHelped.status = "OK"
        idleTask = Task("IDLE", self.taskPrios["IDLE"], robotThatWasHelped.name, 0, 0, 0, False, " ")
        idleTaskMsg = idleTask.convertTaskToTaskMsg()
        robotThatWasHelped.currentTask = idleTaskMsg
        robotThatWasHelped.currentTaskName = idleTaskMsg.taskName
        self.idleTask(idleTaskMsg)

        # determine which operator this came from, set it to not helping
        operatorThatWasHelping = self.activeOperatorDictionary[robotThatWasHelped.operatorID]
        operatorThatWasHelping.operatorIsBusy = False
        operatorThatWasHelping.unlinkFromRobot(robotThatWasHelped)

        for robot in self.activeRobotDictionary.values():
            # check for IDLE robots
            if robot.currentTaskName == "HLP" and (robot.operatorID == operatorThatWasHelping.operatorID):
                # unlink operator from helping robot
                operatorThatWasHelping.unlinkFromHelpingRobot(robot)
                self.setRobotToIdle(robot)
                break

        if len(self.robotsForOperator) > 0:
            robotToHelp = self.robotsForOperator.pop()
            print("Sending new robot: ", robotToHelp.name, " ,to operator.")
            operatorThatWasHelping.sendNewRobotToOperator.publish(robotToHelp)
            operatorThatWasHelping.operatorIsBusy = True
            operatorThatWasHelping.linkToRobot(robotToHelp)

    def releaseFromHelp(self, data):
        robotThatWasHelping = self.activeRobotDictionary[data.data]
        operatorThatWasUsingTheRobot = self.activeOperatorDictionary[robotThatWasHelping.operatorID]
        operatorThatWasUsingTheRobot.unlinkFromHelpingRobot(robotThatWasHelping)
        operatorThatWasUsingTheRobot.needHelpingRobot = False
        self.setRobotToIdle(robotThatWasHelping)

    def removeFromPriorityQueue(self, data):
        ID = data.data
        print("ID: ", data.data)
        for supervisor in self.activeSupervisorDictionary.values():
            for i, task in enumerate(supervisor.taskPriorityQueue):
                if (task.getID() == ID):
                    supervisor.taskPriorityQueue.remove(task)

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
        robotToNavigateTo = self.activeRobotDictionary[data.data]

        # find out which operator this robot came from
        operatorAskingForHelp = self.activeOperatorDictionary[robotToNavigateTo.operatorID]
        operatorAskingForHelp.needHelpingRobot = True

        # grab the pose of the robot to navigate to
        robotToNavigateToPose = robotToNavigateTo.pose.pose.pose.position
        robotToNavigateToOrient = robotToNavigateTo.pose.pose.pose.orientation
        quat_list = [robotToNavigateToOrient.x, robotToNavigateToOrient.y, robotToNavigateToOrient.z,
                     robotToNavigateToOrient.w]
        robotEuler = euler_from_quaternion(quat_list)
        robYaw = robotEuler[2]
        xBuffer = np.sin(robYaw) * 1
        yBuffer = np.cos(robYaw) * 1
        # print(robotEuler)
        robotToNavigateToX = robotToNavigateToPose.x - xBuffer
        robotToNavigateToY = robotToNavigateToPose.y - yBuffer

        supervisorWhosRobotIsStuck = self.activeSupervisorDictionary[robotToNavigateTo.supervisorID]
        info_str = ""
        # send it to the supervisor that issued a stuck request if the supervisor has more than 1 robot to work with
        if (len(supervisorWhosRobotIsStuck.activeRobotDictionary.values()) > 1):
            info_str = supervisorWhosRobotIsStuck.supervisorID + ' ' + 'HLP' + ' ' + 'unassigned ' + str(
                robotToNavigateToX) + ' ' + str(
                robotToNavigateToY) + ' ' + str(robYaw + (7 * np.pi / 4)) + ' ' + 'False' + ' '
        else:
            for supervisor in self.activeSupervisorDictionary.values():
                if (len(supervisor.activeRobotDictionary.values()) > 1):
                    info_str = supervisor.supervisorID + ' ' + 'HLP' + ' ' + 'unassigned ' + str(
                        robotToNavigateToX) + ' ' + str(
                        robotToNavigateToY) + ' ' + str(robYaw + (7 * np.pi / 4)) + ' ' + 'False' + ' '
        print(info_str)
        self.processTask(info_str)

    def sendTaskCodesToSupervisor(self, req):
        """
        A boot message that provides color display infor and task type to the supervisor UI
        :param req:
        :return:
        """
        # colors are IDL-grey Nav-green DLV-orange Help-red CLN-blue SOS-red SEARCH-pink
        taskCodeStringList = ['IDLE Idle False #9BA8AB True', 'NAV Navigation True #75D858 True',
                              'DLV Delivery True #B27026 True', 'HLP Help True #A600FF False',
                              'CLN Clean True #00A2FF True', 'SOS Stuck False #FF0000 True',
                              'SEARCH Search False #FFC0CB True']
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
        newRobot.operatorID = ""
        newRobot.supervisorID = ""

        initialTask = Task("IDLE", self.taskPrios["IDLE"], newRobot.name, 0, 0, 0, False, " ", " ")
        initialTaskMsg = initialTask.convertTaskToTaskMsg()
        newRobot.currentTaskName = initialTaskMsg.taskName
        newRobot.currentTask = initialTaskMsg
        self.unsupervisedTasks.append(initialTask)
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

    def registerNewOperator(self, data):
        print("registering new operator: " + data.data)
        newOperator = Operator(data.data)
        self.activeOperatorDictionary[data.data] = newOperator
        # Check the robotsForOperatorQueue to see if there are robots that need help
        if (len(self.robotsForOperator) > 0):
            robotToHelp = self.robotsForOperator.pop()
            newOperator.sendNewRobotToOperator.publish(robotToHelp)
            newOperator.operatorIsBusy = True
            newOperator.linkToRobot(robotToHelp)
            print("Passing robot: " + robotToHelp.name + " to operator: " + newOperator.operatorID)

    def registerNewSupervisor(self, data):
        print("registering new supervisor: " + data.data)
        newSupervisor = Supervisor(data.data)
        # if a brand new supervisor exists and its the only, give them the tasks of the robots to start
        if (len(self.activeSupervisorDictionary.values()) == 0):
            # record the OG ID
            for task in self.unsupervisedTasks:
                task.OGSupervisorID = newSupervisor.supervisorID
            for robot in self.activeRobotDictionary.values():
                robot.supervisorID = newSupervisor.supervisorID
                newSupervisor.activeRobotDictionary[robot.name] = robot
            newSupervisor.taskPriorityQueue = self.unsupervisedTasks
            del self.unsupervisedTasks[:]
            self.activeSupervisorDictionary[data.data] = newSupervisor

        else:
            print('Scatter the fleet')
            # handle distribution of robots here
            self.activeSupervisorDictionary[data.data] = newSupervisor
            self.distributeRobotsToSupervisor()

    def unregisterSupervisor(self, data):
        # remove the object from the dictionary
        print(data.data + " died an honorable death.")
        supervisor = self.activeSupervisorDictionary[data.data]
        # handle distribution of robots here
        self.distributeDeadPriorityQueue(supervisor.taskPriorityQueue)
        del self.activeSupervisorDictionary[data.data]
        del supervisor
        self.distributeRobotsToSupervisor()

        print("Unregistered " + data.data)

    def unregisterOperator(self, data):
        # remove the object from the dictionary
        print(data.data + " died an honorable death.")
        operator = self.activeOperatorDictionary[data.data]
        # check to see if the operator is working on any robots
        if (operator.currentRobot is not None):
            #   put it back into the robots for operator queue, don't make it IDLE. It still needs help
            self.robotsForOperator.append(operator.currentRobot)
            self.robotsForOperator.sort(key=lambda robot: robot.currentTask.taskPriority)
            #   unlink them from the robot
            operator.unlinkFromRobot(operator.currentRobot)
        # if there's a helping robot, unlink them from that, make it IDLE
        if (operator.currentHelpingRobot is not None):
            robotThatWasHelping = operator.currentHelpingRobot
            operatorThatWasUsingTheRobot = self.activeOperatorDictionary[robotThatWasHelping.operatorID]
            operatorThatWasUsingTheRobot.unlinkFromHelpingRobot(robotThatWasHelping)
            operatorThatWasUsingTheRobot.needHelpingRobot = False
            self.setRobotToIdle(robotThatWasHelping)

        del operator
        del self.activeOperatorDictionary[data.data]
        print("Unregistered " + data.data)

    # Called when # Supervisors changes (register or unregister)
    def distributeRobotsToSupervisor(self):
        # go through the robots and assign one at a time
        supervisorList = list(self.activeSupervisorDictionary.values())
        if (len(supervisorList) == 0):
            for i, robot in enumerate(self.activeRobotDictionary.values()):
                robot.supervisorID = ''
            return
        for supervisor in self.activeSupervisorDictionary.values():
            supervisor.activeRobotDictionary = {}
            del supervisor.activeTaskList[:]
        for i, robot in enumerate(self.activeRobotDictionary.values()):
            supervisor = supervisorList[(i % len(self.activeSupervisorDictionary.values()))]
            supervisor.activeRobotDictionary[robot.name] = robot
            robot.supervisorID = supervisor.supervisorID
            robotTask = self.convertTaskMsgToTask(robot.currentTask)
            supervisor.activeTaskList.append(robotTask)

    # I think this will work?
    def convertTaskMsgToTask(self, taskMsg):
        # calculate basePriority
        timeElapsed = (time.time() - taskMsg.timeAdded) / 60
        basePriority = self.taskPrios[taskMsg.taskName]

        task = Task(taskMsg.taskName, basePriority, taskMsg.robotName,
                    taskMsg.X, taskMsg.Y, taskMsg.yaw, taskMsg.isRaisedPriority, taskMsg.OGSupervisorID,
                    taskMsg.variables)
        task.addedToQueue = taskMsg.timeAdded
        return task

        # TODO: have to distribute the tasks in the priorityQueue to the other superviors and the activeTaskList...
        #  clear activeTaskList. When we place a robot in a supervisors dictionary (after clearing it), we convert the
        #  taskmsg to task and add it to activeTaskList for that supervisor. When a new supervisor is created, don't
        #  shuffle around the PQ. If a supervisor was killed, we take their priority queue, pass it into the function.
        #  And then for each of the rest of the supervisors, round robin convert to task object and append to
        #  their respective PQs

    # Called when a supervisor unregisters to give other active supervisors the tasks
    def distributeDeadPriorityQueue(self, orphanedPQ):
        # if no supervisors are left dump it into the middleman unsupervised tasks and make all active robots idle
        if len(self.activeSupervisorDictionary) == 1:  # supervisor gets deleted after this function (process task uses supervisor)
            for task in orphanedPQ:
                task.OGSupervisorID = ''
                self.unsupervisedTasks.append(task)
            for robot in self.activeRobotDictionary.values():
                self.setRobotToIdle(robot)
                robot.supervisorID = ''
        else:
            supervisorList = list(self.activeSupervisorDictionary.values())
            for i, task in enumerate(orphanedPQ):
                supervisor = supervisorList[(i % len(self.activeSupervisorDictionary.values()))]
                supervisor.taskPriorityQueue.append(task)

    def setRobotToIdle(self, robot):
        # check for if not assigned to a supervisor
        if robot.supervisorID == '':
            return  # just return because the robot should already be Idle
        info_str = robot.supervisorID + ' ' + 'IDLE' + ' ' + robot.name + ' ' + '0' + ' ' + '0' + ' ' + '0' + ' ' + 'False' + ' '
        self.robot_stuck_count[robot.name] = 0
        # print("Setting Robot " + str(robot.name) + " count to 0")
        self.processTask(info_str)

    def findOperatorFromRobot(self, robot):
        for operator in self.activeOperatorDictionary.values():
            if (operator.currentRobot == robot):
                return operator

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

        self.statePublisherForOperator.publish(robotList)
        self.statePublisherForSupervisor.publish(robotList)

    def assignIdleRobots(self):
        """
        Checks for idle robots if there are tasks in the priority queue. Assigns them the tasks so little to no downtime
        until all tasks are completed.
        """
        for robot in self.activeRobotDictionary.values():
            # check for IDLE robots
            if robot.currentTaskName == "IDLE" and len(self.sc.search_list) == 0:
                if (self.activeRobotDictionary[robot.name].supervisorID != ""):
                    supervisor = self.activeSupervisorDictionary[self.activeRobotDictionary[robot.name].supervisorID]
                    if len(supervisor.taskPriorityQueue) > 0:
                        # assign robot a task in th priority queue
                        highestPriorityTask = supervisor.taskPriorityQueue.pop()
                        highestPriorityTask.robotName = robot.name
                        taskMsg = highestPriorityTask.convertTaskToTaskMsg()

                        # print(self.activeTaskList)
                        for task in supervisor.activeTaskList:
                            if robot.currentTask.ID == task.getID():
                                supervisor.activeTaskList.remove(task)
                                break
                        # print(self.activeTaskList)

                        robot.currentTask = taskMsg
                        robot.currentTaskName = taskMsg.taskName
                        self.taskFns[highestPriorityTask.taskName](taskMsg)
                        supervisor.activeTaskList.append(highestPriorityTask)

    def publishRobotsLeftInQueue(self):
        """
        Publishes the length of the robot operator help queue
        """
        self.robotsLeftInQueue.publish(len(self.robotsForOperator))

    def publishTaskList(self):
        """
        Publishes the task list (active tasks being completed and unassigned priority sorted tasks)
        """
        for supervisor in self.activeSupervisorDictionary.values():
            taskMsgList = TaskMsgArr()
            supervisor.taskPriorityQueue.sort(key=lambda t: t.getPriority())
            for task in (supervisor.activeTaskList + supervisor.taskPriorityQueue):
                # turn object into Msg type to publish
                taskMsg = task.convertTaskToTaskMsg()
                taskMsgList.taskMsgs.append(taskMsg)
            supervisor.taskListPublisher.publish(taskMsgList)

    # When a given amount of time has passed, check for reassignment of robot to highest priority task in the priority
    # queue
    def dynamicReassignmentCheck(self):
        """
        When a given amount of time has passed, check for reassignment of robot to highest priority task in the priority
        queue
        """
        fiveMinutes = 60
        for supervisor in self.activeSupervisorDictionary.values():
            # if the time has elapsed for a reassinment check, and the length of both queues are greater than zero (meaning we can swap)
            if ((time.time() - self.reassignmentCounter > fiveMinutes) and (len(supervisor.taskPriorityQueue) > 0) and (
                    len(supervisor.activeTaskList) > 0)):
                highestPriorityTask = supervisor.taskPriorityQueue[-1]
                supervisor.activeTaskList.sort(key=lambda task: task.getPriority())
                lowestPriorityTask = supervisor.activeTaskList[0]
                if (lowestPriorityTask.taskName != "IDLE"):
                    if highestPriorityTask.getPriority() > lowestPriorityTask.getPriority():
                        # print("Reassigning Task " + str(highestPriorityTask.taskName) + ": " + str(highestPriorityTask.getPriority()))
                        swappedTasks = TaskMsgArr()
                        # swap the active task with high  [Active, High Priority]
                        swappedTasks.taskMsgs.append(lowestPriorityTask.convertTaskToTaskMsg())
                        swappedTasks.taskMsgs.append(highestPriorityTask.convertTaskToTaskMsg())
                        supervisor.taskReassignmentPublisher.publish(swappedTasks)
                        # remove from priority queue
                        supervisor.taskPriorityQueue.pop()
                        # reassign robot name
                        highestPriorityTask.robotName = lowestPriorityTask.robotName
                        # add new task to active
                        supervisor.activeTaskList.append(highestPriorityTask)
                        self.taskFns[highestPriorityTask.taskName](highestPriorityTask)  # call task function

                        # remove now inactive from activeList
                        supervisor.activeTaskList.remove(lowestPriorityTask)
                        robotBeingReassigned = supervisor.activeRobotDictionary[lowestPriorityTask.robotName]
                        robotBeingReassigned.currentTask = highestPriorityTask.convertTaskToTaskMsg()
                        robotBeingReassigned.currentTaskName = highestPriorityTask.taskName
                        # reassign active task to unassigned status
                        lowestPriorityTask.robotName = 'unassigned'
                        # add previously active task to priority queue
                        supervisor.taskPriorityQueue.append(lowestPriorityTask)

                self.reassignmentCounter = time.time()

    def checkNavStatus(self):
        posTolerance = .3
        orientationTolerance = 0.1
        for robot in self.activeRobotDictionary.values():
            if (robot.currentTask.taskName == "NAV"):
                quat = robot.pose.pose.pose.orientation
                quat_list = [quat.x, quat.y, quat.z, quat.w]
                robotEuler = euler_from_quaternion(quat_list)
                robYaw = robotEuler[2]

                if ((abs(robot.pose.pose.pose.position.x - robot.currentTask.X) <= posTolerance) and
                        (abs(robot.pose.pose.pose.position.y - robot.currentTask.Y) <= posTolerance) and
                        (abs(robYaw - robot.currentTask.yaw) <= orientationTolerance)):
                    # if we do via points, instead of IDLE task, send next position in via points list
                    self.setRobotToIdle(robot)

            pass

    def checkSearchStatus(self):

        guard_searching = (self.search_algorithm==0)
        guard_clustering = (self.search_algorithm==1)

        posTolerance = .3
        moveTolerance = 0.000005

        robots = self.activeRobotDictionary.keys()

        # for robot in self.activeRobotDictionary.values():
        for robotName in robots:

            robot = self.activeRobotDictionary.get(robotName)
            if (robot.currentTask.taskName == "SEARCH"):
                quat = robot.pose.pose.pose.orientation
                quat_list = [quat.x, quat.y, quat.z, quat.w]
                robotEuler = euler_from_quaternion(quat_list)
                robYaw = robotEuler[2]

                # If the guard was reached
                if ((abs(robot.pose.pose.pose.position.x - robot.currentTask.X) <= posTolerance) and
                        (abs(robot.pose.pose.pose.position.y - robot.currentTask.Y) <= posTolerance)):
                    # if we do via points, instead of IDLE task, send next position in via points list
                    self.setRobotToIdle(robot)

                    if guard_searching or guard_clustering:
                        self.sc.mark_guard_searched(self.guardDictionary.get(robotName))

                elif self.robot_stuck_count.get(robotName) is not None:
                    if self.robot_stuck_count.get(robotName) > 100:
                        self.setRobotToIdle(robot)

                        # TODO: change this to sc.reassign_guard() so that it isn't marked searched
                        #       was only done this way to prevent impossible to reach guards from crashing the system

                        print("MM: Robot %s is stuck, giving up on node" %(str(robotName)))
                        if guard_searching:
                            self.sc.reassign_guard(self.guardDictionary.get(robotName))


                if self.robot_previous_poses.get(robotName) is not None:
                    prev_pose = self.robot_previous_poses.get(robotName)
                    if ((abs(robot.pose.pose.pose.position.x - prev_pose[0]) <= moveTolerance) and
                            (abs(robot.pose.pose.pose.position.y - prev_pose[1]) <= moveTolerance) and
                            (abs(robYaw - prev_pose[2]) <= moveTolerance)):
                        if self.robot_stuck_count.get(robotName) is not None:
                            count = self.robot_stuck_count.get(robotName)
                            self.robot_stuck_count[robotName] = count+1
                    else:
                        if self.robot_stuck_count.get(robotName) is not None:
                            self.robot_stuck_count[robotName] = 0


                # rospy.logwarn(robotName + " position is " + str(robot.pose.pose.pose.position.x) + ", " + str(robot.pose.pose.pose.position.y) + ", " + str(robYaw))

                self.robot_previous_poses[robotName] = (robot.pose.pose.pose.position.x, robot.pose.pose.pose.position.y, robYaw)

            pass

    def get_force_vector(self, thisRobotName):
        robots = self.activeRobotDictionary.keys()
        thisRobot = self.activeRobotDictionary.get(thisRobotName)
        force = np.array([[0.0],[0.0]])
        for robotName in robots:
            robot = self.activeRobotDictionary.get(robotName)
            if robotName != thisRobotName:
                quat = robot.pose.pose.pose.orientation
                quat_list = [quat.x, quat.y, quat.z, quat.w]
                robotEuler = euler_from_quaternion(quat_list)
                robYaw = robotEuler[2]

                dx = thisRobot.pose.pose.pose.position.x - robot.pose.pose.pose.position.x
                dy = thisRobot.pose.pose.pose.position.y - robot.pose.pose.pose.position.y
                robots_theta = math.atan2(dy, dx)
                radius = math.sqrt(dx**2+dy**2)

                force += np.array([[math.cos(robots_theta)],[math.sin(robots_theta)]])*float(1/radius)

        scale = 10
        force = force*scale
        new_pose = np.array([[thisRobot.pose.pose.pose.position.x + force[0][0]],[thisRobot.pose.pose.pose.position.y+force[1][0]]])
        # print(new_pose)

        # res = self.map.info.resolution
        # hi_x = abs(self.map.info.origin.position.x)
        # hi_y = abs(self.map.info.origin.position.y)
        #
        # x = (np.interp(new_pose[0][0], [-hi_x, hi_x], [0, (hi_x * 2)])) / res
        # y = (np.interp(new_pose[1][0], [-hi_y, hi_y], [0, (hi_y * 2)])) / res
        # map_coords = int(y *self.map.info.width +x)
        # map_coords = int(new_pose[1][0] * self.map.info.width + new_pose[0][0])
        map_coords = self.point_to_index((new_pose[0][0], new_pose[1][0]))
        # print(map_coords)

        # map_coords = int(((new_pose[1][0] + self.map.info.origin.position.y)*(1/res))*self.map.info.width + (new_pose[0][0] + self.map.info.origin.position.x)*(1/res))

        if self.map.data[map_coords] == -1 or self.map.data[map_coords] == 100:
            rospy.logwarn("force in obstacle or unknown")
            reduced_force = force/10
            for i in range(10):
                incremental_force = reduced_force*i
                new_pose = np.array([[thisRobot.pose.pose.pose.position.x + incremental_force[0][0]],
                                     [thisRobot.pose.pose.pose.position.y + incremental_force[1][0]]])
                # map_coords = int(new_pose[1][0] * self.map.info.width + new_pose[0][0])
                map_coords = self.point_to_index((new_pose[0][0], new_pose[1][0]))
                if self.map.data[map_coords] == 0:
                    return new_pose
            rospy.logwarn("could not find incremental force point that wasnt in obstacle")
        else:
            rospy.logwarn("force sending to open cell")

        return new_pose


    def point_to_index(self, loc):

        res = self.map.info.resolution
        Xorigin = self.map.info.origin.position.x
        Yorigin = self.map.info.origin.position.y

        Xcell = int((loc[0] - Xorigin - (res / 2)) / res)
        Ycell = int((loc[1] - Yorigin - (res / 2)) / res)

        return Ycell*self.map.info.width + Xcell


    def get_random_point(self, thisRobotName):
        thisRobot = self.activeRobotDictionary.get(thisRobotName)

        new_pose = np.array([[thisRobot.pose.pose.pose.position.x+(np.random.uniform(0,3)*(1.0 if np.random.random() < 0.5 else -1.0))],[thisRobot.pose.pose.pose.position.y+(np.random.uniform(0,3)*(1.0 if np.random.random() < 0.5 else -1.0))]])

        map_coords = self.point_to_index((new_pose[0][0], new_pose[1][0]))
        if self.map.data[map_coords] == -1 or self.map.data[map_coords] == 100:
            # print("finding another random point - previous was unknown or occupied")
            return self.get_random_point(thisRobotName)
        else:
            return new_pose


    def do_searching(self):
        if   (self.search_algorithm == 0): # Guard searching
            self.guard_searching()
        elif (self.search_algorithm == 1): # Guard clustering
            self.guard_searching()
        elif (self.search_algorithm == 2): # Force dispersion
            self.classical_searching("force_dispersion")
        elif (self.search_algorithm == 3): # Random walk
            middleman.classical_searching("random_walk")
        else:
            pass


    def classical_searching(self, name):

        sup = list(self.activeSupervisorDictionary.keys())
        found_tolerance = 3 #set to realsense depth
        if len(self.activeSupervisorDictionary.values()) != 0:

            if not self.searchStarted:
                self.start_time = rospy.Time.now().secs
                self.items_list = ["scissors", "advil", "bandages"]
                self.sc.start_search(self.items_list)
                self.searchStarted = True
                print("STARTED SEARCH")
                print(str(self.searchStarted))
                self.item_list_world_coords = {}
                for item_name in self.items_list:
                    item_closest_guard = self.sc.item_location_dict.get(item_name)
                    item = self.transform_map_to_realworld((item_closest_guard.x, item_closest_guard.y),
                                                           self.sc.get_width_and_height())
                    self.item_list_world_coords[item_name] = item
                    print(item_name + " position is " + str(item))

            if len(self.items_list) == 0:
                rospy.loginfo("SEARCH DONE")

            else:
                robots = self.activeRobotDictionary.keys()
                for robotName in robots:
                    robot = self.activeRobotDictionary.get(robotName)
                    for item_name in self.items_list:
                        item = self.item_list_world_coords.get(item_name)
                        if abs(item[0]-robot.pose.pose.pose.position.x) < found_tolerance and abs(item[1]-robot.pose.pose.pose.position.y) < found_tolerance:
                            print("MML " + item_name.upper() + " FOUND")
                            print("Time found: " + str(rospy.Time.now().secs - self.start_time) + " seconds")

                            self.items_list.remove(item_name)

                    if (robot.currentTaskName == "IDLE") and (len(self.activeSupervisorDictionary.values()) != 0):

                        if name == "force_dispersion":
                            print("MM: new force for " + robotName)
                            new_position = self.get_force_vector(robotName)
                        elif name == "random_walk":
                            print("MM: new random point for " + robotName)
                            new_position = self.get_random_point(robotName)

                        quat_list = [0, 0, 0, 1]
                        robotEuler = euler_from_quaternion(quat_list)
                        yaw = robotEuler[2]
                        taskMsg = Task("SEARCH", 0, robotName, new_position[0][0], new_position[1][0], yaw, False,
                                       sup[0]).convertTaskToTaskMsg()
                        robot.currentTask = taskMsg
                        robot.currentTaskName = taskMsg.taskName
                        self.sendRobotToPos(robot,  new_position[0][0], new_position[1][0], yaw)

    def guard_searching(self):

        if len(self.activeRobotDictionary) > 0:

            wh = self.sc.get_width_and_height()
            robots = self.activeRobotDictionary.keys()

            for robotName in robots:
                robot = self.activeRobotDictionary.get(robotName)

                if (robot.currentTaskName == "IDLE") and (len(self.activeSupervisorDictionary.values()) != 0) and (
                        self.sc.get_num_nodes_to_search() > 0):
                    x = robot.pose.pose.pose.position.x
                    y = robot.pose.pose.pose.position.y
                    quat = robot.pose.pose.pose.orientation
                    quat_list = [quat.x, quat.y, quat.z, quat.w]
                    robotEuler = euler_from_quaternion(quat_list)
                    yaw = robotEuler[2]

                    #if x != 0 and y != 0:
                    sup = list(self.activeSupervisorDictionary.keys())

                    print("MM: Robot %s is no longer idle, sending to guard" % (str(robotName)))

                    # rospy.logwarn("robot " + str(robotName) + " position is: " + str(x) + ", " + str(y))
                    pt_to_search = self.transform_realworld_to_map((x, y), wh)
                    # rospy.logwarn("robot " + str(robotName) + " trap graph position is " + str(pt_to_search[0]) + ", " + str(pt_to_search[1]))

                    a_guard, guard_position = self.sc.get_guard_to_search((pt_to_search[0], pt_to_search[1]), robot=robotName)

                    if a_guard is None:
                        print("MM: got no node, setting robot %s to idle" % (str(robotName)))
                        self.setRobotToIdle(robot)

                    else:
                        # rospy.logwarn("guard: " + str(guard_position))
                        self.guardDictionary[robotName] = a_guard
                        map_pt = self.transform_map_to_realworld((guard_position[0], guard_position[1]), wh)
                        # self.sendRobotToPos(robot, map_pt[0], map_pt[1])
                        taskMsg = Task("SEARCH", 0, robotName, map_pt[0], map_pt[1], yaw, False,
                                        sup[0]).convertTaskToTaskMsg()
                        robot.currentTask = taskMsg
                        robot.currentTaskName = taskMsg.taskName
                        self.sendRobotToPos(robot, map_pt[0], map_pt[1], yaw)

                    # else:
                    #     rospy.loginfo("Robot " + robotName + " is " + robot.currentTaskName + " and " + str(len(self.activeSupervisorDictionary.values())) + " supervisor")
                    #     rospy.loginfo("Robot " + robotName + " task location: " + str(robot.currentTask.X) + ", " + str(robot.currentTask.Y))

                # Lets hope it never has a guard at 0,0
                if (robot.currentTaskName == "SEARCH") and (robot.currentTask.X == 0) and (robot.currentTask.Y == 0):
                    self.setRobotToIdle(robot)
                #if (robot.currentTaskName == "IDLE") and (robot.currentTask.X == 0) and (robot.currentTask.Y == 0):
                #    self.setRobotToIdle(robot)

    def transform_realworld_to_map(self, pt, trap_graph_wh):

        transformed_pt = []
        res = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        trap_graph_width = float(trap_graph_wh[0])
        trap_graph_height = float(trap_graph_wh[1])
        map_width = float(self.map.info.width)
        map_height = float(self.map.info.height)
        # print(map_width)
        # print(trap_graph_width)
        translated_x = (pt[0] - origin_x) / res
        translated_y = -(pt[1] - origin_y) / res + map_height
        # rospy.logwarn("intermediate point is " + str([translated_x, translated_y]))
        scaled_x = translated_x * (trap_graph_width / map_width)
        scaled_y = translated_y * (trap_graph_height / map_height)
        # rospy.logwarn("scaled point is " + str([scaled_x, scaled_y]))
        # GUARD IS ROW, COL
        transformed_pt.append(int(scaled_x))
        transformed_pt.append(int(scaled_y))
        # rospy.logwarn("transformed point world to guard is: " + str(transformed_pt))
        # transformed_pt = (0, 0)
        return transformed_pt

    def transform_map_to_realworld(self, pt, trap_graph_wh):
        transformed_pt = []
        res = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        trap_graph_width = float(trap_graph_wh[0])
        trap_graph_height = float(trap_graph_wh[1])
        map_width = float(self.map.info.width)
        map_height = float(self.map.info.height)

        scaled_x = pt[0] * (map_width / trap_graph_width)
        scaled_y = pt[1] * (map_height / trap_graph_height)
        # scaled_y = pt[1]-map_height*(map_height/trap_graph_height)

        # transformed_pt.append(scaled_x*res + origin_x)
        # transformed_pt.append(-(scaled_y*res + origin_y))
        transformed_pt.append(scaled_x * res + origin_x)
        transformed_pt.append(-(scaled_y - map_height) * res + origin_y)

        # rospy.logwarn("transformed point guard to world is: " + str(transformed_pt))
        # transformed_pt = (0,0)
        return transformed_pt

    def map_callback(self, msg):
        self.map = msg


'''
Parameters for coordinated search
 0 = guard searching
 1 = guard clustering
 2 = force dispersion
 3 = random walk
'''
search_mode = 3

middleman = Middleman(search_mode)

while not rospy.is_shutdown():
    middleman.assignIdleRobots()
    middleman.publishRobotStates()
    middleman.publishRobotsLeftInQueue()
    middleman.publishTaskList()
    middleman.dynamicReassignmentCheck()
    middleman.checkSearchStatus()
    middleman.do_searching()
    middleman.rate.sleep()
