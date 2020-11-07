#!/usr/bin/env python3
import rospy
import rospkg
from PyQt5 import QtCore, QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QGraphicsScene, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsTextItem, \
    QGraphicsView, QHeaderView, QTableWidgetItem, QListView, QGraphicsLineItem
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, qRgb, QTransform, QFont, QWheelEvent, QStandardItemModel
from PyQt5.QtCore import QThread, pyqtSignal, QPoint, QPointF
from PyQt5.QtCore import Qt, QLineF, QRectF
import sys
import json
import time
import math
from std_msgs.msg import *
from supervisorUI.msg import Robot, RobotArr, TaskMsg, TaskMsgArr
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from supervisorUI.srv import TaskString, TaskStringResponse
rospack = rospkg.RosPack()
designerFile = rospack.get_path('supervisorUI')+"/src/SupervisorUI.ui"
map = rospack.get_path('supervisorUI')+"/src/Maps/Hospital"

class SupervisorUI(QtWidgets.QMainWindow):
    robotUpdateSignal = pyqtSignal('PyQt_PyObject')
    taskUpdateSignal = pyqtSignal('PyQt_PyObject')
    robotTaskChangeSignal = pyqtSignal('PyQt_PyObject')
    def __init__(self, width, height):
        super(SupervisorUI, self).__init__()
        self.ui = uic.loadUi(designerFile, self)#loads the ui file and adds member names to class
        self.slideInMenuWidth = 0 # these all get set by fitToScreen method
        self.windowHeight = 0
        self.windowWidth = 0
        self.mapWidth = 0
        self.numAnimationSteps = 20
        self.fitToScreen(width, height)
        self.RobotShapes = []
        self.LocationTargetShapes = []
        self.RobotNames = []
        self.RobotTasks = {}
        self.RobotTasksToCode = {}
        self.robotUpdateSignal.connect(self.drawRobotCallback)
        self.robotTaskChangeSignal.connect(self.taskChangeMainThreadCallback)
        self.taskUpdateSignal.connect(self.taskListCallback)
        self.stateSubscriber = rospy.Subscriber('/supervisor/robotState', RobotArr, self.robotStateCallback)
        self.taskSubscriber = rospy.Subscriber('/supervisor/taskList', TaskMsgArr, self.taskListRosCallback)
        self.taskChangeSubscriber = rospy.Subscriber('/supervisor/taskReassignment', TaskMsgArr, self.taskChangeCallback)
        self.taskPublisher = rospy.Publisher("/supervisor/task", String, queue_size=10)
        rospy.wait_for_service('/supervisor/taskCodes')
        self.fillRobotTasks()
        self.SideMenuThread = SideMenuThread()
        self.SideMenuThread.signal.connect(self.SideMenuThreadCallback)
        self.pickLocation = False
        self.LocationPicked = False
        self.LocationCoordinates = (0, 0)
        self.CreateTaskButton.clicked.connect(self.ToggleSideMenu)
        self.SelectLocationBTN.clicked.connect(self.SelectLocationCallback)
        self.ConfirmTask.clicked.connect(self.confirmTaskCreationCallback)
        self.TaskSwitchOKBTN.clicked.connect(self.okBTNCallback)
        self.SelectTaskCB.activated.connect(self.taskComboBoxChanged)
        #subscriber for reciecing robot locations on the map
        #graphicsSize = self.SupervisorMap.size()
        self.black = QColor(qRgb(0, 0, 0))
        self.blue = QColor(qRgb(30, 144, 255))
        self.red = QColor(qRgb(220, 20, 60))
        self.green = QColor(qRgb(0, 255, 127))
        self.Orange = QColor(qRgb(255, 165, 0))
        self.yellow = QColor(qRgb(255, 255, 0))
        self.purple = QColor(qRgb(238, 130, 238))
        self.magenta = QColor(qRgb(255, 0, 255))
        self.white = QColor(qRgb(255, 255, 255))
        self.scaleFactor = 0.4*self.mapWidth/1500 #the map was designed for a 1500 pixel square map.This adjusts for the screen size
        self.scene = QGraphicsScene()
        self.scene.mousePressEvent = self.mapClickEventHandler  # allows for the grid to be clicked
        self.SupervisorMap.setMouseTracking(True)
        #todo self.scene.mouseMoveEvent = self.MouseMovementEvent
        self.SupervisorMap.wheelEvent = self.wheelEvent
        self.SupervisorMap.setScene(self.scene)
        self.loadMap(map)
        self.SideMenuShowing = False
        self.SetUpTable()

    def ToggleSideMenu(self):
        if not self.SideMenuShowing:
            self.ErrorLBL.hide()
            self.populateTaskComboBox()
            self.taskComboBoxChanged()
            self.SelectRobot.clear()
            self.SelectRobot.addItem("unassigned")
            for item in self.RobotNames:
                self.SelectRobot.addItem(item)
            self.XLocLabel.setText("X:")
            self.YLocLabel.setText("Y:")
            self.LocationPicked = False
        else:
            if self.LocationPicked:
                for item in self.LocationTargetShapes:
                    self.scene.removeItem(item)
        self.SideMenuThread.start()

    def SideMenuThreadCallback(self, result):
        if(self.SideMenuShowing):
            self.CreateTaskButton.setText("Create Task")
            self.SupervisorMap.resize((self.mapWidth-self.slideInMenuWidth)+(result+1)*self.slideInMenuWidth/self.numAnimationSteps, self.windowHeight)
            self.SideMenu.move(self.windowWidth-self.slideInMenuWidth+(result+1)*self.slideInMenuWidth/self.numAnimationSteps, 0)
            if(result+1==20):
                self.SideMenuShowing = False
        else:
            self.CreateTaskButton.setText("Close")
            self.SupervisorMap.resize(self.mapWidth - (result + 1) * self.slideInMenuWidth/self.numAnimationSteps, self.windowHeight)
            self.SideMenu.move(self.windowWidth - (result + 1) * self.slideInMenuWidth/self.numAnimationSteps, 0)
            if (result + 1 == 20):
                self.SideMenuShowing = True

    def keyPressEvent(self, event):#this is overriding qMainWindow
        if event.key() == Qt.Key_M:
            pass

    def SetUpTable(self):
        self.RobotListTable.setColumnCount(3)
        self.RobotListTable.setHorizontalHeaderLabels(('Robot ID', 'Task', 'Status'))
        self.RobotListTable.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)

        self.PriorityQueueTable.setColumnCount(3)
        self.PriorityQueueTable.setHorizontalHeaderLabels(('Task', 'ID', 'Priority'))
        self.PriorityQueueTable.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.PriorityQueueTable.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.PriorityQueueTable.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)


    def loadMap(self, mapLocation):
        #These shapes just help make the viewport more centered since it autocenters around the bounding box of the map objects
        #shape0 = QGraphicsRectItem(-1500, -1000, 1, 1)
        #shape0.setPen(self.white)
        #self.scene.addItem(shape0)
        #shape1 = QGraphicsRectItem(1500, 1500, 1, 1)
        #shape1.setPen(self.white)
        #self.scene.addItem(shape1)
        #########################################################
        with open(mapLocation) as file:
            mapObjList = json.load(file)
        for item in mapObjList["objects"]:
            if(item["type"]=="rect"):
                shape = QGraphicsRectItem(item["centerX"]-item["length"]/2, -item["centerY"]-item["width"]/2, item["length"], item["width"])
                shape.setTransformOriginPoint(QPoint(item["centerX"], -item["centerY"]))
                shape.setPen(QPen(self.black))
                shape.setBrush(QBrush(self.black, Qt.SolidPattern))
                shape.setRotation(item["rotation"])
                self.scene.addItem(shape)
            elif (item["type"]=="text"):
                label = QGraphicsTextItem(item["text"])
                label.setX(item["centerX"] - item["length"] / 2)
                label.setY(-item["centerY"] - item["width"] / 2)
                font = QFont("Bavaria")
                font.setPointSize(24)
                font.setWeight(QFont.Bold)
                label.setFont(font)
                label.setRotation(item["rotation"])
                self.scene.addItem(label)
        self.SupervisorMap.scale(self.scaleFactor, self.scaleFactor)

    def wheelEvent(self, event):
        pointBeforeScale = self.SupervisorMap.mapToScene(event.pos())
        oldScale = self.scaleFactor
        self.SupervisorMap.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        if event.angleDelta().y()>0:
            if(self.scaleFactor < 2.0):
                self.scaleFactor += 0.01*self.mapWidth/1500
        elif event.angleDelta().y()<0:
            if (self.scaleFactor > 0.4*self.mapWidth/1500):
                self.scaleFactor -= 0.01*self.mapWidth/1500
        self.SupervisorMap.scale(self.scaleFactor/oldScale, self.scaleFactor/oldScale)
        #self.SupervisorMap.setTransformationAnchor(QGraphicsView.NoAnchor)
        pointAfterScale = self.SupervisorMap.mapToScene(event.pos())
        offset = pointAfterScale-pointBeforeScale
        center = self.SupervisorMap.mapToScene(self.SupervisorMap.viewport().rect().center())
        #self.SupervisorMap.setViewport(QPointF(center+offset))
        print(center+offset)
        print(center)
        print(" ")
        self.SupervisorMap.centerOn(center-offset)

    def robotStateCallback(self, message):
        #print("MSG received")
        self.robotUpdateSignal.emit(message)

    def drawRobotCallback(self, result):
        self.RobotListTable.setRowCount(0)#clear table
        for item in self.RobotShapes:
            self.scene.removeItem(item)
        self.RobotShapes.clear()
        self.RobotNames.clear()
        for item in result.robots:
            self.RobotListTable.insertRow(self.RobotListTable.rowCount())
            self.RobotListTable.setItem(self.RobotListTable.rowCount()-1, 0, QTableWidgetItem(item.name))
            self.RobotListTable.setItem(self.RobotListTable.rowCount()-1, 1, QTableWidgetItem(item.currentTaskName))
            self.RobotListTable.setItem(self.RobotListTable.rowCount() - 1, 2, QTableWidgetItem(item.status))
            obSize = 70
            shape = QGraphicsEllipseItem(int(item.pose.pose.pose.position.x*100 - obSize / 2), -int(item.pose.pose.pose.position.y*100 + obSize / 2), obSize, obSize)
            shape.setPen(QPen(self.black))
            color = QColor(self.RobotTasks[item.currentTaskName][2])
            shape.setBrush(QBrush(color, Qt.SolidPattern))
            self.scene.addItem(shape)
            self.RobotShapes.append(shape)
            self.RobotNames.append(item.name)
            label = QGraphicsTextItem(item.name)
            label.setX(int(item.pose.pose.pose.position.x*100))
            label.setY(-int(item.pose.pose.pose.position.y*100+obSize*1.2))
            font = QFont("Bavaria")
            font.setPointSize(18)
            font.setWeight(QFont.Bold)
            label.setDefaultTextColor(color)
            label.setFont(font)
            self.scene.addItem(label)
            self.RobotShapes.append(label)
            quat = item.pose.pose.pose.orientation
            siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            #item.pose.pose.pose.orientation.z = item.pose.pose.pose.orientation.z+3.14/2
            #making the arrow
            startArrowX = int(item.pose.pose.pose.position.x*100-math.cos(yaw)*obSize/4)
            startArrowY = -int(item.pose.pose.pose.position.y*100-math.sin(yaw)*obSize/4)
            endArrowX =int(item.pose.pose.pose.position.x*100+math.cos(yaw)*obSize/2-math.cos(yaw)*obSize/4)
            endArrowY = -int(item.pose.pose.pose.position.y*100+math.sin(yaw)*obSize/2-math.sin(yaw)*obSize/4)
            line = QGraphicsLineItem(startArrowX,
                                     startArrowY,
                                     endArrowX,
                                    endArrowY)
            line.setPen(QPen(self.black, 5))
            self.scene.addItem(line)
            self.RobotShapes.append(line)
            line = QGraphicsLineItem(endArrowX,
                                    endArrowY,
                                     endArrowX-math.cos(3.14/4+yaw)*obSize/4,
                                     endArrowY+math.sin(3.14/4+yaw)*obSize/4)

            line.setPen(QPen(self.black, 5))
            self.scene.addItem(line)
            self.RobotShapes.append(line)
            line = QGraphicsLineItem(endArrowX,
                                    endArrowY,
                                     endArrowX- math.cos(3.14 / 4 - yaw) * obSize / 4,
                                     endArrowY- math.sin(3.14 / 4 - yaw) * obSize / 4)

            line.setPen(QPen(self.black, 5))
            self.scene.addItem(line)
            self.RobotShapes.append(line)

    def taskListRosCallback(self, message):
        self.taskUpdateSignal.emit(message)

    def taskListCallback(self, result):
        self.PriorityQueueTable.setRowCount(0)  # clear table
        for item in result.taskMsgs:
            if item.robotName == "unassigned":
                self.PriorityQueueTable.insertRow(self.PriorityQueueTable.rowCount())
                self.PriorityQueueTable.setItem(self.PriorityQueueTable.rowCount() - 1, 0, QTableWidgetItem(item.taskName))
                self.PriorityQueueTable.setItem(self.PriorityQueueTable.rowCount() - 1, 1, QTableWidgetItem(item.ID))
                self.PriorityQueueTable.setItem(self.PriorityQueueTable.rowCount() - 1, 2, QTableWidgetItem("{:.2f}".format(item.taskPriority)))

    def fillRobotTasks(self):
        self.RobotTasks = {}
        self.RobotTasksToCode = {}
        getTasksFromMiddleMan = rospy.ServiceProxy('/supervisor/taskCodes', TaskString)
        response = getTasksFromMiddleMan("YOOOOO")
        for item in response.tasks:
            taskInfo = item.split()
            self.RobotTasks[taskInfo[0]] = (taskInfo[1], taskInfo[2], taskInfo[3])
            self.RobotTasksToCode[taskInfo[1]] = taskInfo[0]

    def populateTaskComboBox(self):
        #will eventually use service request for tasks
        self.SelectTaskCB.clear()
        for key in self.RobotTasks.keys():
            self.SelectTaskCB.addItem(self.RobotTasks[key][0])

    def SelectLocationCallback(self):
        self.SelectLocationBTN.setEnabled(False)
        self.pickLocation = True

    def mapClickEventHandler(self, event):
        if self.pickLocation:
            if self.LocationPicked:
                for item in self.LocationTargetShapes:
                    self.scene.removeItem(item)
            clickedX = event.scenePos().x()
            clickedY = event.scenePos().y()
            self.LocationCoordinates = (round(clickedX/100, 3), round(-clickedY/100, 3))
            self.XLocLabel.setText("X: "+"{:.2f}".format(clickedX/100))
            self.YLocLabel.setText("Y: " + "{:.2f}".format(-clickedY/100))
            ##########################Draws Target on screen with 3 circles
            obSize = 50
            shape = QGraphicsEllipseItem(int(clickedX - obSize / 2),
                                         int(clickedY - obSize / 2), obSize, obSize)
            shape.setPen(QPen(self.red))
            shape.setBrush(QBrush(self.red, Qt.SolidPattern))
            self.LocationTargetShapes.append(shape)
            self.scene.addItem(shape)

            obSize = 40
            shape = QGraphicsEllipseItem(int(clickedX - obSize / 2),
                                         int(clickedY - obSize / 2), obSize, obSize)
            shape.setPen(QPen(self.white))
            shape.setBrush(QBrush(self.white, Qt.SolidPattern))
            self.LocationTargetShapes.append(shape)
            self.scene.addItem(shape)

            obSize = 30
            shape = QGraphicsEllipseItem(int(clickedX - obSize / 2),
                                         int(clickedY - obSize / 2), obSize, obSize)
            shape.setPen(QPen(self.red))
            shape.setBrush(QBrush(self.red, Qt.SolidPattern))
            self.LocationTargetShapes.append(shape)
            self.scene.addItem(shape)
            #####################################################################
            self.pickLocation = False
            self.LocationPicked = True
            self.SelectLocationBTN.setEnabled(True)

    def confirmTaskCreationCallback(self):
        #Task Strings: 'task_name robot_name X Y [vars ...]'
        if self.RobotTasks[self.RobotTasksToCode[self.SelectTaskCB.currentText()]][1]=="True":
            if not self.LocationPicked:
                self.ErrorLBL.show()
            else:
                self.taskPublisher.publish(self.RobotTasksToCode[
                                               self.SelectTaskCB.currentText()] + " " + self.SelectRobot.currentText() + " " + str(
                    self.LocationCoordinates[0]) + " " + str(self.LocationCoordinates[1]))
                if self.LocationPicked:
                    for item in self.LocationTargetShapes:
                        self.scene.removeItem(item)
                self.ErrorLBL.hide()
                self.populateTaskComboBox()
                self.taskComboBoxChanged()
                self.SelectRobot.clear()
                self.SelectRobot.addItem("unassigned")
                for item in self.RobotNames:
                    self.SelectRobot.addItem(item)
                self.XLocLabel.setText("X:")
                self.YLocLabel.setText("Y:")
                self.LocationPicked = False
        else:
            self.taskPublisher.publish(self.RobotTasksToCode[self.SelectTaskCB.currentText()] + " " + self.SelectRobot.currentText()+ " " + str(self.LocationCoordinates[0])+ " "+ str(self.LocationCoordinates[1]))
            if self.LocationPicked:
                for item in self.LocationTargetShapes:
                    self.scene.removeItem(item)
            self.ErrorLBL.hide()
            self.populateTaskComboBox()
            self.taskComboBoxChanged()
            self.SelectRobot.clear()
            self.SelectRobot.addItem("unassigned")
            for item in self.RobotNames:
                self.SelectRobot.addItem(item)
            self.XLocLabel.setText("X:")
            self.YLocLabel.setText("Y:")
            self.LocationPicked = False

    def taskChangeCallback(self, message):
        self.robotTaskChangeSignal.emit(message)

    def taskChangeMainThreadCallback(self, result):
        changes = result.taskMsgs
        messageString = changes[0].robotName + " Reassigned to "+changes[1].taskName +" " + changes[1].ID+". Previous task was "+changes[0].taskName+" " + changes[0].ID
        self.TaskSwitchList.addItem(messageString)
        self.TaskSwitchFrame.show()

    def okBTNCallback(self):
        self.TaskSwitchList.clear()
        self.TaskSwitchFrame.hide()

    def taskComboBoxChanged(self):
        #print("Combo changed")
        self.ErrorLBL.hide()
        for item in self.LocationTargetShapes:
            self.scene.removeItem(item)
        if(self.RobotTasks[self.RobotTasksToCode[self.SelectTaskCB.currentText()]][1]=="True"):
            self.SelectLocationBTN.setEnabled(True)
        else:
            self.SelectLocationBTN.setEnabled(False)
            self.LocationCoordinates = (0, 0)
            self.XLocLabel.setText("X: ")
            self.YLocLabel.setText("Y: ")
            self.LocationPicked = False

    def fitToScreen(self, width, height):
        #The app should have the same aspect ratio regardless of the computer's
        #aspect ratio
        desAspX = 16
        desAspY = 9
        if width/height>desAspX/desAspY:
            self.windowHeight = int(height*0.9)
            self.windowWidth = int(self.windowHeight*desAspX/desAspY)
        else:
            self.windowWidth = int(width*0.9)
            self.windowHeight = int(self.windowWidth*desAspY/desAspX)
        self.setFixedSize(self.windowWidth, self.windowHeight)
        self.slideInMenuWidth = self.windowWidth*0.2
        robotListWidth = self.windowWidth*0.2
        self.mapWidth = self.windowWidth*0.8
        createTaskButtonWidth = self.windowWidth*0.12
        createTaskButtonHeight = self.windowHeight*0.02
        self.RobotListTable.resize(robotListWidth, self.windowHeight*0.7)
        self.priorityQueueLBL.move(0,self.windowHeight*0.7)
        self.priorityQueueLBL.resize(robotListWidth, self.windowHeight * 0.02)
        self.PriorityQueueTable.move(0,self.windowHeight*0.72)
        self.PriorityQueueTable.resize(robotListWidth, self.windowHeight * 0.28)
        self.SupervisorMap.resize(self.mapWidth, self.windowHeight)
        self.SupervisorMap.move(robotListWidth, 0)
        self.CreateTaskButton.resize(createTaskButtonWidth, createTaskButtonHeight)
        self.CreateTaskButton.move(self.windowWidth*0.96-createTaskButtonWidth, self.windowHeight*0.98-createTaskButtonHeight)
        self.SideMenu.move(self.windowWidth, 0)
        self.SideMenu.resize(self.slideInMenuWidth, self.windowHeight)

        self.TaskCreatorLabel.move(self.slideInMenuWidth*0.05, self.windowHeight*0.01)
        self.TaskCreatorLabel.resize(self.slideInMenuWidth*0.9, self.windowHeight*0.04)
        self.SelectTaskLabel.move(self.slideInMenuWidth*0.05, self.windowHeight*0.05)
        self.SelectTaskLabel.resize(self.slideInMenuWidth * 0.9, self.windowHeight * 0.02)
        self.SelectTaskCB.move(self.slideInMenuWidth*0.05, self.windowHeight*0.07)
        self.SelectTaskCB.resize(self.slideInMenuWidth * 0.9, self.windowHeight * 0.03)

        self.AssignedRobotTXTLabel.move(self.slideInMenuWidth * 0.05, self.windowHeight * 0.11)
        self.AssignedRobotTXTLabel.resize(self.slideInMenuWidth * 0.9, self.windowHeight * 0.02)
        self.SelectRobot.move(self.slideInMenuWidth*0.05, self.windowHeight*0.14)
        self.SelectRobot.resize(self.slideInMenuWidth * 0.90, self.windowHeight * 0.04)

        self.SelectLocationBTN.move(self.slideInMenuWidth*0.05, self.windowHeight*0.19)
        self.SelectLocationBTN.resize(self.slideInMenuWidth * 0.45, self.windowHeight * 0.04)
        self.XLocLabel.move(self.slideInMenuWidth*0.52, self.windowHeight*0.19)
        self.XLocLabel.resize(self.slideInMenuWidth*0.45, self.windowHeight*0.02)
        self.YLocLabel.move(self.slideInMenuWidth*0.52, self.windowHeight*0.21)
        self.YLocLabel.resize(self.slideInMenuWidth*0.45, self.windowHeight*0.02)

        self.ConfirmTask.move(self.slideInMenuWidth*0.05, self.windowHeight*0.24)
        self.ConfirmTask.resize(self.slideInMenuWidth*0.9, self.windowHeight*0.04)

        self.ErrorLBL.resize(self.slideInMenuWidth*0.9, self.windowHeight*0.02)
        self.ErrorLBL.move(self.slideInMenuWidth*0.05, self.windowHeight*0.28)

        self.TaskSwitchFrame.hide()
        TaskFrameWidth = self.windowWidth*0.6
        TaskFrameHeight = self.windowHeight*0.1
        self.TaskSwitchFrame.resize(TaskFrameWidth, TaskFrameHeight)
        self.TaskSwitchFrame.move(self.windowWidth*0.2, self.windowHeight*0.01)
        self.TaskSwitchList.resize(TaskFrameWidth*0.9-3, TaskFrameHeight*0.8-3)
        self.TaskSwitchList.move(3, TaskFrameHeight*0.2)#the border is 3px wide
        self.TaskSwitchOKBTN.resize(TaskFrameWidth*0.1-3, TaskFrameHeight-6)
        self.TaskSwitchOKBTN.move(TaskFrameWidth*0.9, 3)
        self.TaskSwitchInfo.resize(TaskFrameWidth*0.9-3, TaskFrameHeight*0.2-3)
        self.TaskSwitchInfo.move(3, 3)

class SideMenuThread(QThread):
    signal = pyqtSignal('PyQt_PyObject')
    def __init__(self):
        QThread.__init__(self)
        self.numSec = 0.2
        self.numSteps = 20
    def run(self):
        for x in range(0, self.numSteps):
            time.sleep(self.numSec/self.numSteps)
            self.signal.emit(x)

if __name__ == '__main__':
    rospy.init_node('SupervisorUI')
    rospy.sleep(.5)
    app = QApplication(sys.argv)
    screen = app.primaryScreen()
    size = screen.size()
    window = SupervisorUI(size.width(), size.height())
    window.show()
    sys.exit(app.exec_())