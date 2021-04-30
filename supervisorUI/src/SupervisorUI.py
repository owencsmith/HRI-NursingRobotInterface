#!/usr/bin/env python3
import rospy
import rospkg
from PyQt5 import QtCore, QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QGraphicsScene, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsTextItem, \
    QGraphicsView, QHeaderView, QTableWidgetItem, QListView, QGraphicsLineItem, QPushButton, QGraphicsItemGroup, \
    QGraphicsPolygonItem
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, qRgb, QTransform, QFont, QWheelEvent, QStandardItemModel, QIcon, \
    QImage, QPixmap, QRgba64, QPolygonF
from PyQt5.QtCore import QThread, pyqtSignal, QPoint, QPointF
from PyQt5.QtCore import Qt, QLineF, QRectF
import sys
import json
import time
import math
from std_msgs.msg import *
from middleman.msg import Robot, RobotArr, TaskMsg, TaskMsgArr
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point32
from geometry_msgs.msg import Polygon as RosPolygon
from middleman.srv import TaskString, TaskStringResponse, PolygonList, PolygonListResponse
from sensor_msgs.msg import Image

rospack = rospkg.RosPack()
designerFile = rospack.get_path('supervisorUI') + "/src/SupervisorUI.ui"
map = rospack.get_path('supervisorUI') + "/src/Maps/Hospital.json"
trashIcon = rospack.get_path('supervisorUI') + "/src/trashCanIcon.jpg"
cameraIcon = rospack.get_path('supervisorUI') + "/src/camera-icon.png"


def QPoint_to_Point32(qpt):
    """
    Convert from a QT QPoint to a ROS Point32
    :param qpt: QT QPoint
    :return: pt: ROS Point32
    """
    pt = Point32()
    pt.x = round(qpt.x() / 100, 3)
    pt.y = -round(qpt.y() / 100, 3)
    pt.z = 0
    return pt


class SupervisorUI(QtWidgets.QMainWindow):
    robotUpdateSignal = pyqtSignal('PyQt_PyObject')
    taskUpdateSignal = pyqtSignal('PyQt_PyObject')
    robotTaskChangeSignal = pyqtSignal('PyQt_PyObject')
    CamUpdateSignal = pyqtSignal('PyQt_PyObject')

    def __init__(self, width, height, id):
        super(SupervisorUI, self).__init__()
        self.ui = uic.loadUi(designerFile, self)  # loads the ui file and adds member names to class
        self.slideInMenuWidth = 0  # these all get set by fitToScreen method
        self.windowHeight = 0
        self.id = id
        self.windowWidth = 0
        self.mapWidth = 0
        self.numAnimationSteps = 20
        self.fitToScreen(width, height)
        self.NavWaypoints = []
        self.RobotShapes = []
        self.LocationTargetShapes = []
        self.AllRobotNames = []
        self.SupervisorSpecificRobotNames = []
        self.RobotTasks = {}
        self.GoalTarget = []
        self.ObstacleList = []
        self.RoomNameList = []
        self.RobotTasksToCode = {}
        self.taskListRefresh = 0
        self.sketchMode = False
        self.poseYaw = 90
        self.poseShapes = []
        self.PoseSlider.setMinimum(0)
        self.PoseSlider.setMaximum(360)
        self.PoseSlider.valueChanged.connect(self.updatePoseLBL)
        self.PoseSlider.setValue(0)
        self.robotUpdateSignal.connect(self.drawRobotCallback)
        self.robotTaskChangeSignal.connect(self.taskChangeMainThreadCallback)
        self.taskUpdateSignal.connect(self.taskListCallback)
        self.CamUpdateSignal.connect(self.CamUpdate)
        self.stateSubscriber = rospy.Subscriber('/supervisor/robotState', RobotArr, self.robotStateCallback)
        self.taskSubscriber = rospy.Subscriber('/' + id + '/taskList', TaskMsgArr, self.taskListRosCallback)
        self.taskChangeSubscriber = rospy.Subscriber('/' + id + '/taskReassignment', TaskMsgArr,
                                                     self.taskChangeCallback)
        self.taskPublisher = rospy.Publisher("/supervisor/task", String, queue_size=10)
        self.deleteTaskPublisher = rospy.Publisher("/supervisor/removeTask", String, queue_size=10)
        self.CamSubscriber = rospy.Subscriber('none', Image, self.CamRosCallback)
        rospy.wait_for_service('/supervisor/taskCodes')

        self.fillRobotTasks()
        self.SideMenuThread = SideMenuThread()
        self.SideMenuThread.signal.connect(self.SideMenuThreadCallback)
        self.pickLocation = False
        self.LocationPicked = False
        self.clickedX = 0
        self.clickedY = 0
        self.LocationCoordinates = (0, 0)

        # connect buttons and other Qt modules to action events
        self.CreateTaskButton.clicked.connect(self.ToggleSideMenu)
        self.SelectLocationBTN.clicked.connect(self.SelectLocationCallback)
        self.ConfirmTask.clicked.connect(self.confirmTaskCreationCallback)
        self.TaskSwitchOKBTN.clicked.connect(self.okBTNCallback)
        self.SelectTaskCB.activated.connect(self.taskComboBoxChanged)
        self.PriorityQueueTable.cellClicked.connect(self.taskListClickedCallback)
        self.ToggleObstaclesCB.stateChanged.connect(self.obstacleComboBoxChanged)
        self.CloseCameraBTN.clicked.connect(self.closeCam)
        self.RobotListTable.cellClicked.connect(self.openCam)
        self.SketchToggleButton.clicked.connect(self.sketchModeToggleClickedCallback)
        self.RectangleSketchButton.clicked.connect(self.rectangleSketchButtonCallback)
        self.FreeHandSketchButton.clicked.connect(self.freeHandSketchButtonCallback)
        self.LineSketchButton.clicked.connect(self.lineSketchButtonCallback)
        self.HighRiskSketchButton.clicked.connect(self.highRiskSketchButtonCallback)
        self.MediumRiskSketchButton.clicked.connect(self.mediumRiskSketchButtonCallback)
        self.LowRiskSketchButton.clicked.connect(self.lowRiskSketchButtonCallback)

        # colors for ease of use
        self.black = QColor(qRgb(0, 0, 0))
        self.blue = QColor(qRgb(30, 144, 255))
        self.red = QColor(qRgb(220, 20, 60))
        self.redFill = QColor(QRgba64.fromRgba(220, 20, 60, 75))
        self.green = QColor(qRgb(65, 215, 125))
        self.greenFill = QColor(QRgba64.fromRgba(65, 215, 125, 75))
        self.orange = QColor(qRgb(255, 165, 0))
        self.orangeFill = QColor(QRgba64.fromRgba(255, 165, 0, 75))
        self.yellow = QColor(qRgb(255, 255, 0))
        self.purple = QColor(qRgb(238, 130, 238))
        self.magenta = QColor(qRgb(255, 0, 255))
        self.white = QColor(qRgb(255, 255, 255))
        self.gray = QColor(qRgb(119, 136, 153))

        self.scaleFactor = 0.4 * self.mapWidth / 1500  # the map was designed for a 1500 pixel square map.This adjusts for the screen size
        self.scene = QGraphicsScene()
        self.scene.mousePressEvent = self.mapClickEventHandler  # allows for the grid to be clicked
        self.SupervisorMap.setMouseTracking(True)
        self.scene.mouseMoveEvent = self.mapMouseMoveEventHandler
        self.scene.mouseReleaseEvent = self.mapMouseReleaseEventHandler
        self.SupervisorMap.wheelEvent = self.wheelEvent
        self.SupervisorMap.setScene(self.scene)
        self.loadMap(map)
        self.scene.setSceneRect(self.SupervisorMap.sceneRect())

        # for keeping track of sketching positions
        self.last_x = None
        self.last_y = None
        self.left_click = False
        self.right_click = False
        self.graphics_group = self.scene.createItemGroup([])
        self.sketchItems = []
        self.drawingModality = 0
        self.sketchPen = QPen(self.red)
        self.sketchBrush = QBrush(self.redFill)
        self.FreeHandSketchButton.setEnabled(False)
        self.HighRiskSketchButton.setEnabled(False)
        self.firstPolygonPoint = None
        self.polygonList = []

        self.sketchButtonList = [self.RectangleSketchButton, self.FreeHandSketchButton, self.LineSketchButton,
                                 self.LowRiskSketchButton, self.MediumRiskSketchButton, self.HighRiskSketchButton]
        for buttons in self.sketchButtonList:
            buttons.setVisible(False)

        self.SideMenuShowing = False
        self.SetUpTable()

        self.sketchChanged = False  # this will get set to true whenever edit sketch is turned off
        self.sketchObstacleServer = rospy.Service('/supervisor/sketchObstacles', PolygonList, self.sendSketchObstacles)

        # New supervisor message for middleman
        self.newSupervisorIDPublisher = rospy.Publisher('/supervisor/new_supervisor_ui', String, queue_size=10)

        # heartbeat stuff
        rospy.Timer(rospy.Duration(.5), self.sendHeartBeat)
        self.heartBeatPublisher = rospy.Publisher('/' + id + '/heartbeat', String, queue_size=10)
        rospy.sleep(1)
        self.newSupervisorIDPublisher.publish(id)

    def sendHeartBeat(self, data):
        self.heartBeatPublisher.publish("lubdub")

    def ToggleSideMenu(self):
        if not self.SideMenuShowing:
            self.ErrorLBL.hide()
            self.populateTaskComboBox()
            self.taskComboBoxChanged()
            self.SelectRobot.clear()
            self.SelectRobot.addItem("unassigned")
            for item in self.SupervisorSpecificRobotNames:
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
        if (self.SideMenuShowing):
            self.CreateTaskButton.setText("Create Task")
            self.SupervisorMap.resize(
                (self.mapWidth - self.slideInMenuWidth) + (result + 1) * self.slideInMenuWidth / self.numAnimationSteps,
                self.windowHeight)
            self.SideMenu.move(self.windowWidth - self.slideInMenuWidth + (
                        result + 1) * self.slideInMenuWidth / self.numAnimationSteps, 0)
            if (result + 1 == 20):
                self.SideMenuShowing = False
        else:
            self.CreateTaskButton.setText("Close")
            self.SupervisorMap.resize(self.mapWidth - (result + 1) * self.slideInMenuWidth / self.numAnimationSteps,
                                      self.windowHeight)
            self.SideMenu.move(self.windowWidth - (result + 1) * self.slideInMenuWidth / self.numAnimationSteps, 0)
            if (result + 1 == 20):
                self.SideMenuShowing = True

    def keyPressEvent(self, event):  # this is overriding qMainWindow
        if event.key() == Qt.Key_M:
            pass

    def SetUpTable(self):
        self.RobotListTable.setColumnCount(4)
        self.RobotListTable.setHorizontalHeaderLabels(('Robot ID', 'Task', 'Status', ''))
        self.RobotListTable.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.RobotListTable.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.PriorityQueueTable.setColumnCount(4)
        self.PriorityQueueTable.setHorizontalHeaderLabels(('Task', 'ID', 'Priority', ''))
        self.PriorityQueueTable.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.PriorityQueueTable.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.PriorityQueueTable.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.PriorityQueueTable.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)

    def loadMap(self, mapLocation):
        with open(mapLocation) as file:
            mapObjList = json.load(file)
        for item in mapObjList["objects"]:
            if (item["type"] == "rect"):
                shape = QGraphicsRectItem(item["centerX"] - item["length"] / 2, -item["centerY"] - item["width"] / 2,
                                          item["length"], item["width"])
                shape.setTransformOriginPoint(QPoint(item["centerX"], -item["centerY"]))
                shape.setPen(QPen(self.black))
                shape.setBrush(QBrush(self.black, Qt.SolidPattern))
                shape.setRotation(item["rotation"])
                self.scene.addItem(shape)
            elif (item["type"] == "text"):
                label = QGraphicsTextItem(item["text"])
                label.setX(item["centerX"] - item["length"] / 2)
                label.setY(-item["centerY"] - item["width"] / 2)
                font = QFont("Bavaria")
                font.setPointSize(24)
                font.setWeight(QFont.Bold)
                label.setFont(font)
                label.setRotation(item["rotation"])
                self.RoomNameList.append(label)
                self.scene.addItem(label)
            elif (item["type"] == "obstacle"):
                shape = QGraphicsRectItem(item["centerX"] - item["length"] / 2, -item["centerY"] - item["width"] / 2,
                                          item["length"], item["width"])
                shape.setTransformOriginPoint(QPoint(item["centerX"], -item["centerY"]))
                shape.setPen(QPen(self.gray))
                shape.setBrush(QBrush(self.gray, Qt.SolidPattern))
                shape.setRotation(item["rotation"])
                self.ObstacleList.append(shape)
            elif (item["type"] == "plant"):
                shape = QGraphicsEllipseItem(item["centerX"] - item["length"] / 2, -item["centerY"] - item["width"] / 2,
                                             item["length"], item["width"])
                shape.setTransformOriginPoint(QPoint(item["centerX"], -item["centerY"]))
                shape.setPen(QPen(self.green))
                shape.setBrush(QBrush(self.green, Qt.SolidPattern))
                shape.setRotation(item["rotation"])
                self.ObstacleList.append(shape)
        self.SupervisorMap.scale(self.scaleFactor, self.scaleFactor)

    def wheelEvent(self, event):
        pointBeforeScale = self.SupervisorMap.mapToScene(event.pos())
        oldScale = self.scaleFactor
        self.SupervisorMap.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        if event.angleDelta().y() > 0:
            if (self.scaleFactor < 2.0):
                self.scaleFactor += 0.01 * self.mapWidth / 1500
        elif event.angleDelta().y() < 0:
            if (self.scaleFactor > 0.4 * self.mapWidth / 1500):
                self.scaleFactor -= 0.01 * self.mapWidth / 1500
        self.SupervisorMap.scale(self.scaleFactor / oldScale, self.scaleFactor / oldScale)
        pointAfterScale = self.SupervisorMap.mapToScene(event.pos())
        offset = pointAfterScale - pointBeforeScale
        center = self.SupervisorMap.mapToScene(self.SupervisorMap.viewport().rect().center())
        self.SupervisorMap.centerOn(center - offset)

    def robotStateCallback(self, message):
        self.robotUpdateSignal.emit(message)

    def drawRobotCallback(self, result):
        self.RobotListTable.setRowCount(0)  # clear table
        for item in self.RobotShapes:
            self.scene.removeItem(item)
        self.RobotShapes.clear()
        self.AllRobotNames.clear()
        self.SupervisorSpecificRobotNames.clear()
        for item in result.robots:
            obSize = 70
            shape = QGraphicsEllipseItem(int(item.pose.pose.pose.position.x * 100 - obSize / 2),
                                         -int(item.pose.pose.pose.position.y * 100 + obSize / 2), obSize, obSize)
            shape.setPen(QPen(self.black))
            # separate the robots by "all" and "supervisor specific"
            if (item.supervisorID == self.id):
                self.SupervisorSpecificRobotNames.append(item.name)
                color = QColor(self.RobotTasks[item.currentTaskName][2])
                shape.setBrush(QBrush(color, Qt.SolidPattern))
                self.RobotListTable.insertRow(self.RobotListTable.rowCount())
                self.RobotListTable.setItem(self.RobotListTable.rowCount() - 1, 0, QTableWidgetItem(item.name))
                self.RobotListTable.setItem(self.RobotListTable.rowCount() - 1, 1,
                                            QTableWidgetItem(item.currentTaskName))
                self.RobotListTable.setItem(self.RobotListTable.rowCount() - 1, 2, QTableWidgetItem(item.status))
                icon_item = QTableWidgetItem()
                icon_item.setIcon(QIcon(cameraIcon))
                self.RobotListTable.setItem(self.RobotListTable.rowCount() - 1, 3, icon_item)
            else:
                color = QColor(self.RobotTasks[item.currentTaskName][2])
                shape.setBrush(QBrush(color, Qt.Dense3Pattern))
            self.scene.addItem(shape)
            self.RobotShapes.append(shape)
            self.AllRobotNames.append(item.name)
            label = QGraphicsTextItem(item.name)
            label.setX(int(item.pose.pose.pose.position.x * 100))
            label.setY(-int(item.pose.pose.pose.position.y * 100 + obSize * 1.2))
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
            # making the arrow
            arrowLines = self.makeArrow(item.pose.pose.pose.position.x * 100, item.pose.pose.pose.position.y * 100, yaw,
                                        obSize)
            for line in arrowLines:
                self.scene.addItem(line)
                self.RobotShapes.append(line)

            type = item.currentTask.taskName
            if self.RobotTasks[type][1] == "True":

                X = item.currentTask.X * 100
                Y = item.currentTask.Y * -100
                shapes = self.makeTarget(X, Y, 50, self.red)
                for shape in shapes:
                    self.RobotShapes.append(shape)
                    self.scene.addItem(shape)
                label = QGraphicsTextItem(item.name + " Goal")
                label.setX(int(X))
                label.setY(int(Y - obSize))
                font = QFont("Bavaria")
                font.setPointSize(18)
                font.setWeight(QFont.Bold)
                label.setDefaultTextColor(self.red)
                label.setFont(font)
                self.scene.addItem(label)
                self.RobotShapes.append(label)

    def taskListRosCallback(self, message):
        self.taskUpdateSignal.emit(message)

    def taskListCallback(self, result):
        self.taskListRefresh += 1
        if self.taskListRefresh == 4:  # this slowsdown the refresh rate so the buttons work more easily
            self.taskListRefresh = 0
            self.PriorityQueueTable.setRowCount(0)  # clear table
            for item in result.taskMsgs:
                if item.robotName == "unassigned":
                    self.PriorityQueueTable.insertRow(self.PriorityQueueTable.rowCount())
                    self.PriorityQueueTable.setItem(self.PriorityQueueTable.rowCount() - 1, 0,
                                                    QTableWidgetItem(item.taskName))
                    self.PriorityQueueTable.setItem(self.PriorityQueueTable.rowCount() - 1, 1,
                                                    QTableWidgetItem(item.ID))
                    self.PriorityQueueTable.setItem(self.PriorityQueueTable.rowCount() - 1, 2,
                                                    QTableWidgetItem("{:.2f}".format(item.taskPriority)))
                    icon_item = QTableWidgetItem()
                    icon_item.setIcon(QIcon(trashIcon))
                    self.PriorityQueueTable.setItem(self.PriorityQueueTable.rowCount() - 1, 3, icon_item)
                    if item.OGSupervisorID != self.id:
                        self.PriorityQueueTable.item(self.PriorityQueueTable.rowCount() - 1, 0).setBackground(
                            QColor(245, 167, 66))
                        self.PriorityQueueTable.item(self.PriorityQueueTable.rowCount() - 1, 1).setBackground(
                            QColor(245, 167, 66))
                        self.PriorityQueueTable.item(self.PriorityQueueTable.rowCount() - 1, 2).setBackground(
                            QColor(245, 167, 66))
                        self.PriorityQueueTable.item(self.PriorityQueueTable.rowCount() - 1, 3).setBackground(
                            QColor(245, 167, 66))

    def sketchModeToggleClickedCallback(self):
        # Toggle Sketch mode on/off
        self.sketchMode = not self.sketchMode
        self.sketchChanged = not self.sketchMode  # Always opposite sketchMode

        if self.sketchMode:
            button_text = 'Save Zones'
            self.SketchToggleButton.setText(button_text)
            for sketch_buttons in self.sketchButtonList:
                sketch_buttons.setVisible(True)
        else:
            button_text = 'Edit Zones'
            self.SketchToggleButton.setText(button_text)

            for sketch_buttons in self.sketchButtonList:
                sketch_buttons.setVisible(False)

    def taskListClickedCallback(self, row, col):
        if col != 3:
            self.drawGoalPointTaskList(row)
        else:
            self.deleteTask(row)

    def deleteTask(self, row):
        id = self.PriorityQueueTable.item(row, 1).text()
        print(id)
        self.deleteTaskPublisher.publish(id)

    def drawGoalPointTaskList(self, row):
        self.removeGoalItem()
        type = self.PriorityQueueTable.item(row, 0).text()
        if self.RobotTasks[type][1] == "True":
            id = self.PriorityQueueTable.item(row, 1).text()
            parts = id.split("_")
            X = float(parts[1]) * 100
            Y = float(parts[2]) * -100
            shapes = self.makeTarget(X, Y, 50, self.blue)
            for shape in shapes:
                self.GoalTarget.append(shape)
                self.scene.addItem(shape)

    def removeGoalItem(self):
        for item in self.GoalTarget:
            self.scene.removeItem(item)
        self.GoalTarget = []

    def fillRobotTasks(self):
        self.RobotTasks = {}
        self.RobotTasksToCode = {}
        getTasksFromMiddleMan = rospy.ServiceProxy('/supervisor/taskCodes', TaskString)
        response = getTasksFromMiddleMan("Wumpus")
        for item in response.tasks:
            taskInfo = item.split()
            self.RobotTasks[taskInfo[0]] = (taskInfo[1], taskInfo[2], taskInfo[3])
            self.RobotTasksToCode[taskInfo[1]] = taskInfo[0]

    def populateTaskComboBox(self):
        # will eventually use service request for tasks
        self.SelectTaskCB.clear()
        for key in self.RobotTasks.keys():
            self.SelectTaskCB.addItem(self.RobotTasks[key][0])

    def SelectLocationCallback(self):
        self.SelectLocationBTN.setEnabled(False)
        self.pickLocation = True

    def mapClickEventHandler(self, event):
        self.removeGoalItem()
        click = event.button()
        if self.pickLocation:
            # if self.LocationPicked:
            #     for item in self.LocationTargetShapes:
            #         self.scene.removeItem(item)
            self.clickedX = event.scenePos().x()
            self.clickedY = event.scenePos().y()
            self.LocationCoordinates = (round(self.clickedX / 100, 3), round(-self.clickedY / 100, 3))
            self.XLocLabel.setText("X: " + "{:.2f}".format(self.clickedX / 100))
            self.YLocLabel.setText("Y: " + "{:.2f}".format(-self.clickedY / 100))
            ##########################Draws Target on screen with 3 circles
            obSize = 50
            shapes = self.makeTarget(self.clickedX, self.clickedY, obSize, self.red)
            for shape in shapes:
                self.LocationTargetShapes.append(shape)
                self.scene.addItem(shape)
            #####################################################################
            self.pickLocation = False
            self.LocationPicked = True
            arrow = self.drawPoseArrow()
            new_label = QtWidgets.QLabel()
            self.WaypointVBox.addWidget(new_label)
            waypoint = NavigationWaypoint(self.clickedX, self.clickedY, self.poseYaw, shapes, arrow, new_label)
            self.NavWaypoints.append(waypoint)
            self.SelectLocationBTN.setEnabled(True)
        elif self.sketchMode:
            if click == QtCore.Qt.LeftButton:
                self.left_click = True
                # We use the graphics group to hold all polygon pieces so we dont create a new one if mid process
                if not self.firstPolygonPoint:
                    self.graphics_group = self.scene.createItemGroup([])
            elif click == QtCore.Qt.RightButton:
                self.right_click = True

    def mapMouseMoveEventHandler(self, event):
        if self.sketchMode:
            pen = self.sketchPen
            pen.setWidth(8)
            brush = self.sketchBrush

            # Polygon drawing - After the first vertex of the polygon is defined,
            # keep redrawing the line from the last point to the current point
            if self.drawingModality == 1 and self.firstPolygonPoint:
                if self.last_x is None:  # This gets set to None everytime a new point is added or a polygon is closed
                    self.last_x = event.scenePos().x()
                    self.last_y = event.scenePos().y()
                else:  # Assumes the last thing added to the sketchItem list was the line
                    self.scene.removeItem(self.sketchItems.pop())

                itemToDraw = QGraphicsLineItem(self.last_x, self.last_y, event.scenePos().x(), event.scenePos().y())
                itemToDraw.setPen(pen)
                self.sketchItems.append(itemToDraw)
                self.scene.addItem(itemToDraw)
                return

            if self.left_click:
                # Freehand drawing
                if self.drawingModality == 0:
                    if self.last_x is None:  # First event.
                        self.last_x = event.scenePos().x()
                        self.last_y = event.scenePos().y()
                        return  # Ignore the first time.

                    itemToDraw = QGraphicsLineItem(self.last_x, self.last_y, event.scenePos().x(), event.scenePos().y())
                    itemToDraw.setPen(pen)
                    self.graphics_group.addToGroup(itemToDraw)

                    # Update the origin for next time.
                    self.last_x = event.scenePos().x()
                    self.last_y = event.scenePos().y()

                # Rectangle drawing
                elif self.drawingModality == 2:
                    if self.last_x is None:
                        self.last_x = event.scenePos().x()
                        self.last_y = event.scenePos().y()
                    else:
                        self.scene.removeItem(self.sketchItems.pop())

                    itemToDraw = QGraphicsRectItem(self.last_x, self.last_y, event.scenePos().x() - self.last_x,
                                                   event.scenePos().y() - self.last_y)
                    itemToDraw.setPen(pen)
                    itemToDraw.setBrush(brush)
                    self.sketchItems.append(itemToDraw)
                    self.scene.addItem(itemToDraw)

            elif self.right_click:
                # erase at mouse intersection
                # check if mouse is over bounding box of any group in sketchItems
                for groups in self.sketchItems:
                    if groups.isUnderMouse():
                        # if it does then remove it from the scene
                        self.scene.removeItem(groups)
                        self.sketchItems.remove(groups)

    def mapMouseReleaseEventHandler(self, event):
        if self.sketchMode:
            # Polygon Drawing
            if self.drawingModality == 1:
                pen = self.sketchPen
                pen.setWidth(8)
                end_click_tol = 15
                if event.button() == QtCore.Qt.LeftButton:
                    newPoint = event.scenePos()
                    self.polygonList.append(newPoint)
                    self.left_click = False

                    d = 25
                    try:
                        itemToDraw = QGraphicsEllipseItem(newPoint.x() - int(d / 2), newPoint.y() - int(d / 2), d, d)
                        itemToDraw.setPen(QPen(self.gray))
                        itemToDraw.setBrush(QBrush(self.gray))
                        self.graphics_group.addToGroup(itemToDraw)
                    except:
                        # do nothing if the ellipse fails to draw
                        # should only occur near the edges of the map if at all
                        pass

                    if not self.firstPolygonPoint:
                        self.firstPolygonPoint = newPoint

                    else:
                        # add the intermediate line
                        self.graphics_group.addToGroup(self.sketchItems.pop())

                        # check for closing of polygon
                        dif = self.firstPolygonPoint-QPointF(newPoint.x(), newPoint.y())
                        if abs(dif.x()) < end_click_tol and abs(dif.y()) < end_click_tol:

                            # create add add the polygon from the points in self.polygonList
                            self.create_fill_polygon(pen)

                            # reset polygon tracking variables
                            self.scene.removeItem(self.graphics_group)
                            self.firstPolygonPoint = None
                            self.polygonList = []

                        self.last_x = None
                        self.last_y = None

                elif event.button() == QtCore.Qt.RightButton and self.firstPolygonPoint:
                    # clear the polygon
                    self.right_click = False
                    # add the intermediate line so it gets removed
                    self.graphics_group.addToGroup(self.sketchItems.pop())
                    self.scene.removeItem(self.graphics_group)
                    self.firstPolygonPoint = None
                    self.polygonList = []

                elif event.button() == QtCore.Qt.RightButton:
                    self.right_click = False

            else:
                if event.button() == QtCore.Qt.LeftButton:
                    self.left_click = False
                    self.sketchItems.append(self.graphics_group)
                elif event.button() == QtCore.Qt.RightButton:
                    self.right_click = False

            self.last_x = None
            self.last_y = None

    def create_fill_polygon(self, pen):
        # for all of the ellipses in the graphics group
        # add the x,y to a QPolygonF
        # create a QGraphicsPolygon
        # color and fill
        # append that to sketch items
        polygon = QPolygonF()
        for point in self.polygonList:
            polygon << point
        polygonToDraw = QGraphicsPolygonItem(polygon)
        polygonToDraw.setPen(pen)
        polygonToDraw.setBrush(self.sketchBrush)
        self.sketchItems.append(polygonToDraw)
        self.scene.addItem(polygonToDraw)

    def makeTarget(self, X, Y, obSize, color):
        shapes = []
        shape1 = QGraphicsEllipseItem(int(X - obSize / 2),
                                      int(Y - obSize / 2), obSize, obSize)
        shape1.setPen(QPen(color))
        shape1.setBrush(QBrush(color, Qt.SolidPattern))

        obSize = obSize - 10
        shape2 = QGraphicsEllipseItem(int(X - obSize / 2),
                                      int(Y - obSize / 2), obSize, obSize)
        shape2.setPen(QPen(self.white))
        shape2.setBrush(QBrush(self.white, Qt.SolidPattern))

        obSize = obSize - 10
        shape3 = QGraphicsEllipseItem(int(X - obSize / 2),
                                      int(Y - obSize / 2), obSize, obSize)
        shape3.setPen(QPen(color))
        shape3.setBrush(QBrush(color, Qt.SolidPattern))
        shapes.append(shape1)
        shapes.append(shape2)
        shapes.append(shape3)

        return shapes

    def makeArrow(self, X, Y, yaw, obSize):
        lines = []
        startArrowX = int(X - math.cos(yaw) * obSize / 4)
        startArrowY = -int(Y - math.sin(yaw) * obSize / 4)
        endArrowX = int(X + math.cos(yaw) * obSize / 2 - math.cos(yaw) * obSize / 4)
        endArrowY = -int(Y + math.sin(yaw) * obSize / 2 - math.sin(yaw) * obSize / 4)
        line1 = QGraphicsLineItem(startArrowX,
                                  startArrowY,
                                  endArrowX,
                                  endArrowY)
        line1.setPen(QPen(self.black, 5))
        lines.append(line1)
        line2 = QGraphicsLineItem(endArrowX,
                                  endArrowY,
                                  endArrowX - math.cos(3.14 / 4 + yaw) * obSize / 4,
                                  endArrowY + math.sin(3.14 / 4 + yaw) * obSize / 4)

        line2.setPen(QPen(self.black, 5))
        lines.append(line2)
        line3 = QGraphicsLineItem(endArrowX,
                                  endArrowY,
                                  endArrowX - math.cos(3.14 / 4 - yaw) * obSize / 4,
                                  endArrowY - math.sin(3.14 / 4 - yaw) * obSize / 4)

        line3.setPen(QPen(self.black, 5))
        lines.append(line3)
        return lines

    def confirmTaskCreationCallback(self):
        if self.RobotTasks[self.RobotTasksToCode[self.SelectTaskCB.currentText()]][1] == "True":
            if not self.LocationPicked:
                self.ErrorLBL.show()
            else:
                yaw = str(math.radians(self.poseYaw))
                self.taskPublisher.publish(self.id + " " + self.RobotTasksToCode[
                    self.SelectTaskCB.currentText()] + " " + self.SelectRobot.currentText() + " " + str(
                    self.LocationCoordinates[0]) + " " + str(self.LocationCoordinates[1]) + " " + yaw + " " + str(
                    self.raisePriorityBox.isChecked()))
                if self.LocationPicked:
                    for item in self.LocationTargetShapes:
                        self.scene.removeItem(item)
                    for item in self.poseShapes:
                        self.scene.removeItem(item)
                self.ErrorLBL.hide()
                self.populateTaskComboBox()
                self.taskComboBoxChanged()
                self.raisePriorityBox.setChecked(False)
                self.SelectRobot.clear()
                self.SelectRobot.addItem("unassigned")
                for item in self.SupervisorSpecificRobotNames:
                    self.SelectRobot.addItem(item)
                self.XLocLabel.setText("X:")
                self.YLocLabel.setText("Y:")
                self.LocationPicked = False
        else:
            yaw = str(math.radians(self.poseYaw))
            self.taskPublisher.publish(self.id + " " + self.RobotTasksToCode[
                self.SelectTaskCB.currentText()] + " " + self.SelectRobot.currentText() + " " +
                                       str(self.LocationCoordinates[0]) + " " + str(
                self.LocationCoordinates[1]) + " " + yaw + " " + str(self.raisePriorityBox.isChecked()))
            if self.LocationPicked:
                for item in self.LocationTargetShapes:
                    self.scene.removeItem(item)
                for item in self.poseShapes:
                    self.scene.removeItem(item)
            self.ErrorLBL.hide()
            self.populateTaskComboBox()
            self.taskComboBoxChanged()
            self.raisePriorityBox.setChecked(False)
            self.SelectRobot.clear()
            self.SelectRobot.addItem("unassigned")
            for item in self.SupervisorSpecificRobotNames:
                self.SelectRobot.addItem(item)
            self.XLocLabel.setText("X:")
            self.YLocLabel.setText("Y:")
            self.LocationPicked = False

    def taskChangeCallback(self, message):
        self.robotTaskChangeSignal.emit(message)

    def taskChangeMainThreadCallback(self, result):
        changes = result.taskMsgs
        messageString = changes[0].robotName + " Reassigned to " + changes[1].taskName + " " + changes[
            1].ID + ". Previous task was " + changes[0].taskName + " " + changes[0].ID
        self.TaskSwitchList.addItem(messageString)
        self.TaskSwitchFrame.show()

    def okBTNCallback(self):
        self.TaskSwitchList.clear()
        self.TaskSwitchFrame.hide()

    def rectangleSketchButtonCallback(self):
        self.drawingModality = 2
        self.RectangleSketchButton.setEnabled(False)
        self.FreeHandSketchButton.setEnabled(True)
        self.LineSketchButton.setEnabled(True)

    def freeHandSketchButtonCallback(self):
        self.drawingModality = 0
        self.RectangleSketchButton.setEnabled(True)
        self.FreeHandSketchButton.setEnabled(False)
        self.LineSketchButton.setEnabled(True)

    def lineSketchButtonCallback(self):
        self.drawingModality = 1
        self.RectangleSketchButton.setEnabled(True)
        self.FreeHandSketchButton.setEnabled(True)
        self.LineSketchButton.setEnabled(False)

    def highRiskSketchButtonCallback(self):
        self.sketchPen = QPen(self.red)
        self.sketchBrush = QBrush(self.redFill)
        self.HighRiskSketchButton.setEnabled(False)
        self.MediumRiskSketchButton.setEnabled(True)
        self.LowRiskSketchButton.setEnabled(True)

    def mediumRiskSketchButtonCallback(self):
        self.sketchPen = QPen(self.orange)
        self.sketchBrush = QBrush(self.orangeFill)
        self.HighRiskSketchButton.setEnabled(True)
        self.MediumRiskSketchButton.setEnabled(False)
        self.LowRiskSketchButton.setEnabled(True)

    def lowRiskSketchButtonCallback(self):
        self.sketchPen = QPen(self.green)
        self.sketchBrush = QBrush(self.greenFill)
        self.HighRiskSketchButton.setEnabled(True)
        self.MediumRiskSketchButton.setEnabled(True)
        self.LowRiskSketchButton.setEnabled(False)

    def taskComboBoxChanged(self):
        # print("Combo changed")
        self.ErrorLBL.hide()
        for item in self.LocationTargetShapes:
            self.scene.removeItem(item)
        if (self.RobotTasks[self.RobotTasksToCode[self.SelectTaskCB.currentText()]][1] == "True"):
            self.SelectLocationBTN.setEnabled(True)
        else:
            self.SelectLocationBTN.setEnabled(False)
            self.LocationCoordinates = (0, 0)
            self.XLocLabel.setText("X: ")
            self.YLocLabel.setText("Y: ")
            self.LocationPicked = False

    def obstacleComboBoxChanged(self):
        if self.ToggleObstaclesCB.isChecked():
            for item in self.RoomNameList:
                self.scene.removeItem(item)
            for item in self.ObstacleList:
                self.scene.addItem(item)
            for item in self.RoomNameList:
                self.scene.addItem(item)
        else:
            for item in self.ObstacleList:
                self.scene.removeItem(item)

    def updatePoseLBL(self):
        self.poseYaw = self.PoseSlider.value() * -1 + 90
        self.PoseDeg.setText(str(self.PoseSlider.value()) + " Deg")
        self.drawPoseArrow()

    def drawPoseArrow(self):
        if self.LocationPicked:
            for line in self.poseShapes:
                self.scene.removeItem(line)
            self.poseShapes = []
            lines = self.makeArrow(self.clickedX, -self.clickedY, math.radians(self.poseYaw), 50)
            for line in lines:
                self.poseShapes.append(line)
                self.scene.addItem(line)
        return lines

    def CamRosCallback(self, result):
        self.CamUpdateSignal.emit(result)

    def CamUpdate(self, result):
        image = QImage(result.data, result.width, result.height, result.step, QImage.Format_RGB888)
        image = image.scaled(self.CameraView.width(), self.CameraView.height(), Qt.KeepAspectRatio)
        pixmap = QPixmap.fromImage(image)
        self.CameraView.setPixmap(pixmap)

    def closeCam(self):
        self.CamSubscriber.unregister()
        self.CameraFrame.hide()

    def openCam(self, row, col):
        if col == 3:
            self.CamSubscriber.unregister()
            name = self.RobotListTable.item(row, 0).text()
            self.CamSubscriber = rospy.Subscriber("/" + name + "/main_cam/color/image_raw", Image,
                                                  self.CamRosCallback)

            self.viewingLBL.setText("Viewing: " + name)
        self.CameraFrame.show()

    def fitToScreen(self, width, height):
        # The app should have the same aspect ratio regardless of the computer's
        # aspect ratio
        desAspX = 16
        desAspY = 9
        if width / height > desAspX / desAspY:
            self.windowHeight = int(height * 0.9)
            self.windowWidth = int(self.windowHeight * desAspX / desAspY)
        else:
            self.windowWidth = int(width * 0.9)
            self.windowHeight = int(self.windowWidth * desAspY / desAspX)
        self.setFixedSize(self.windowWidth, self.windowHeight)
        self.slideInMenuWidth = self.windowWidth * 0.2
        robotListWidth = self.windowWidth * 0.2
        self.mapWidth = self.windowWidth * 0.8
        createTaskButtonWidth = self.windowWidth * 0.12
        zoneButtonWidth = self.windowWidth * 0.07
        thinButtonHeight = self.windowHeight * 0.02

        squareButtonWidth = self.windowWidth * 0.025
        squareButtonHeight = squareButtonWidth
        thinButtonY = self.windowHeight * 0.98 - thinButtonHeight
        squareButtonY = self.windowHeight * 0.98 - squareButtonHeight

        self.RobotListTable.resize(robotListWidth, self.windowHeight * 0.7)
        self.priorityQueueLBL.move(0, self.windowHeight * 0.7)
        self.priorityQueueLBL.resize(robotListWidth, self.windowHeight * 0.02)
        self.PriorityQueueTable.move(0, self.windowHeight * 0.72)
        self.PriorityQueueTable.resize(robotListWidth, self.windowHeight * 0.28)
        self.SupervisorMap.resize(self.mapWidth, self.windowHeight)
        self.SupervisorMap.move(robotListWidth, 0)
        self.ToggleObstaclesCB.move(robotListWidth + self.windowWidth * 0.01, self.windowHeight * 0.96)
        self.ToggleObstaclesCB.resize(createTaskButtonWidth, thinButtonHeight)

        self.CreateTaskButton.resize(createTaskButtonWidth, thinButtonHeight)
        self.SketchToggleButton.resize(zoneButtonWidth, thinButtonHeight)
        self.CreateTaskButton.move(self.windowWidth * 0.96 - createTaskButtonWidth, thinButtonY)
        self.SketchToggleButton.move(self.windowWidth * 0.78 - zoneButtonWidth, thinButtonY)

        self.HighRiskSketchButton.resize(squareButtonWidth, squareButtonHeight)
        self.MediumRiskSketchButton.resize(squareButtonWidth, squareButtonHeight)
        self.LowRiskSketchButton.resize(squareButtonWidth, squareButtonHeight)
        self.HighRiskSketchButton.move(self.windowWidth * 0.62 - squareButtonWidth, squareButtonY)
        self.MediumRiskSketchButton.move(self.windowWidth * 0.65 - squareButtonWidth, squareButtonY)
        self.LowRiskSketchButton.move(self.windowWidth * 0.68 - squareButtonWidth, squareButtonY)

        self.LineSketchButton.resize(squareButtonWidth, squareButtonHeight)
        self.FreeHandSketchButton.resize(squareButtonWidth, squareButtonHeight)
        self.RectangleSketchButton.resize(squareButtonWidth, squareButtonHeight)
        self.FreeHandSketchButton.move(self.windowWidth * 0.42 - squareButtonWidth, squareButtonY)
        self.LineSketchButton.move(self.windowWidth * 0.45 - squareButtonWidth, squareButtonY)
        self.RectangleSketchButton.move(self.windowWidth * 0.48 - squareButtonWidth, squareButtonY)

        self.SideMenu.move(self.windowWidth, 0)
        self.SideMenu.resize(self.slideInMenuWidth, self.windowHeight)
        self.CameraFrame.move(robotListWidth, 0)
        self.CameraFrame.resize(self.windowWidth * 0.3, self.windowWidth * 0.28)
        self.CameraView.move(self.CameraFrame.width() * 0.01, self.CameraFrame.height() * 0.01)
        self.CameraView.resize(self.CameraFrame.width() * 0.98, self.CameraFrame.height() * 0.88)
        self.viewingLBL.move(self.CameraFrame.width() * 0.01, self.CameraFrame.height() * 0.9)
        self.viewingLBL.resize(self.CameraFrame.width() * 0.5, self.CameraFrame.height() * 0.09)
        self.CloseCameraBTN.move(self.CameraFrame.width() * 0.55, self.CameraFrame.height() * 0.9)
        self.CloseCameraBTN.resize(self.CameraFrame.width() * 0.4, self.CameraFrame.height() * 0.09)
        self.CameraFrame.hide()

        self.TaskCreatorLabel.move(self.slideInMenuWidth * 0.05, self.windowHeight * 0.01)
        self.TaskCreatorLabel.resize(self.slideInMenuWidth * 0.9, self.windowHeight * 0.04)
        self.SelectTaskLabel.move(self.slideInMenuWidth * 0.05, self.windowHeight * 0.05)
        self.SelectTaskLabel.resize(self.slideInMenuWidth * 0.9, self.windowHeight * 0.02)
        self.SelectTaskCB.move(self.slideInMenuWidth * 0.05, self.windowHeight * 0.07)
        self.SelectTaskCB.resize(self.slideInMenuWidth * 0.45, self.windowHeight * 0.03)
        self.raisePriorityBox.move(self.slideInMenuWidth * 0.55, self.windowHeight * 0.07)
        self.raisePriorityBox.resize(self.slideInMenuWidth * 0.4, self.windowHeight * 0.03)

        self.AssignedRobotTXTLabel.move(self.slideInMenuWidth * 0.05, self.windowHeight * 0.11)
        self.AssignedRobotTXTLabel.resize(self.slideInMenuWidth * 0.9, self.windowHeight * 0.02)
        self.SelectRobot.move(self.slideInMenuWidth * 0.05, self.windowHeight * 0.14)
        self.SelectRobot.resize(self.slideInMenuWidth * 0.90, self.windowHeight * 0.04)

        self.SelectLocationBTN.move(self.slideInMenuWidth * 0.05, self.windowHeight * 0.19)
        self.SelectLocationBTN.resize(self.slideInMenuWidth * 0.45, self.windowHeight * 0.04)
        self.XLocLabel.move(self.slideInMenuWidth * 0.52, self.windowHeight * 0.19)
        self.XLocLabel.resize(self.slideInMenuWidth * 0.45, self.windowHeight * 0.02)
        self.YLocLabel.move(self.slideInMenuWidth * 0.52, self.windowHeight * 0.21)
        self.YLocLabel.resize(self.slideInMenuWidth * 0.45, self.windowHeight * 0.02)

        self.PoseSlider.move(self.slideInMenuWidth * 0.05, self.windowHeight * 0.24)
        self.PoseSlider.resize(self.slideInMenuWidth * 0.7, self.windowHeight * 0.04)
        self.PoseDeg.move(self.slideInMenuWidth * 0.80, self.windowHeight * 0.24)
        self.PoseDeg.resize(self.slideInMenuWidth * 0.2, self.windowHeight * 0.04)

        self.WaypointScroll.move(self.slideInMenuWidth * 0.05, self.windowHeight * 0.4)
        self.WaypointScroll.resize(self.slideInMenuWidth*0.9, self.windowHeight * 0.5)
        self.WaypointScrollContents.move(self.slideInMenuWidth * 0.05, self.windowHeight * 0.4)
        self.WaypointScrollContents.resize(self.slideInMenuWidth * 0.9, self.windowHeight * 0.5)
        self.WaypointScroll.setFixedWidth(self.slideInMenuWidth*0.9)
        self.WaypointScrollContents.setFixedWidth(self.slideInMenuWidth * 0.9)
        self.WaypointScroll.setAlignment(Qt.AlignHCenter)
        self.WaypointVBox.setAlignment(Qt.AlignHCenter)

        self.ConfirmTask.move(self.slideInMenuWidth * 0.05, self.windowHeight * 0.28)
        self.ConfirmTask.resize(self.slideInMenuWidth * 0.9, self.windowHeight * 0.04)

        self.ErrorLBL.resize(self.slideInMenuWidth * 0.9, self.windowHeight * 0.02)
        self.ErrorLBL.move(self.slideInMenuWidth * 0.05, self.windowHeight * 0.32)

        self.TaskSwitchFrame.hide()
        TaskFrameWidth = self.windowWidth * 0.6
        TaskFrameHeight = self.windowHeight * 0.1
        self.TaskSwitchFrame.resize(TaskFrameWidth, TaskFrameHeight)
        self.TaskSwitchFrame.move(self.windowWidth * 0.2, self.windowHeight * 0.01)
        self.TaskSwitchList.resize(TaskFrameWidth * 0.9 - 3, TaskFrameHeight * 0.8 - 3)
        self.TaskSwitchList.move(3, TaskFrameHeight * 0.2)  # the border is 3px wide
        self.TaskSwitchOKBTN.resize(TaskFrameWidth * 0.1 - 3, TaskFrameHeight - 6)
        self.TaskSwitchOKBTN.move(TaskFrameWidth * 0.9, 3)
        self.TaskSwitchInfo.resize(TaskFrameWidth * 0.9 - 3, TaskFrameHeight * 0.2 - 3)
        self.TaskSwitchInfo.move(3, 3)

    def sendSketchObstacles(self, req):
        updatedSinceLastRequest = int(False)
        obstacles = []
        if self.sketchChanged:
            updatedSinceLastRequest = int(True)
            for item in self.sketchItems:
                print(isinstance(item, QGraphicsPolygonItem))
                if isinstance(item, QGraphicsPolygonItem):
                    p = RosPolygon()
                    # These points are in the scene reference frame
                    gotta_get_them_points = item.polygon()
                    for i in range(gotta_get_them_points.size()):
                        qpoint = gotta_get_them_points[i]
                        pt = QPoint_to_Point32(qpoint)
                        p.points.append(pt)
                    obstacles.append(p)
                elif isinstance(item, QGraphicsRectItem):
                    p = RosPolygon()
                    rect = item.rect()
                    # These points are in the scene reference frame
                    tl = rect.topLeft()
                    tr = rect.topRight()
                    bl = rect.bottomLeft()
                    br = rect.bottomRight()
                    gotta_get_them_points = [tl, tr, br, bl, tl]
                    for i in range(5):
                        qpoint = gotta_get_them_points[i]
                        pt = QPoint_to_Point32(qpoint)
                        p.points.append(pt)
                    obstacles.append(p)
            # print("sso")
            self.sketchChanged = False
        num_obstacles = len(obstacles)

        return PolygonListResponse(obstacles, num_obstacles, updatedSinceLastRequest)


class SideMenuThread(QThread):
    signal = pyqtSignal('PyQt_PyObject')

    def __init__(self):
        QThread.__init__(self)
        self.numSec = 0.2
        self.numSteps = 20

    def run(self):
        for x in range(0, self.numSteps):
            time.sleep(self.numSec / self.numSteps)
            self.signal.emit(x)


class NavigationWaypoint:
    def __init__(self, x, y, yaw, target, arrow, label):

        self._X = x
        self._Y = y
        self._yaw = yaw
        self.target = target
        self.arrow = arrow
        self.Label = label
        self.Label.setText("X: " + "{:.2f}".format(self._X / 100) + " Y: " + "{:.2f}".format(
            -self._Y / 100) + " Yaw: " + str(self._yaw))

    def updateX(self, x):
        self._X = x
        self.Label.setText("X: " + "{:.2f}".format(self._X / 100) + " Y: " + "{:.2f}".format(
            -self._Y / 100) + " Yaw: " + str(self._yaw))

    def updateY(self, y):
        self._Y = y
        self.Label.setText("X: " + "{:.2f}".format(self._X / 100) + " Y: " + "{:.2f}".format(
            -self._Y / 100) + " Yaw: " + str(self._yaw))

    def updateYaw(self, yaw):
        self._yaw = yaw
        self.Label.setText("X: " + "{:.2f}".format(self._X / 100) + " Y: " + "{:.2f}".format(
            -self._Y / 100) + " Yaw: " + str(self._yaw))

if __name__ == '__main__':
    nodeID = "supervisorUI_" + str(int(time.time()))
    rospy.init_node(nodeID)
    rospy.sleep(.5)
    app = QApplication(sys.argv)
    screen = app.primaryScreen()
    size = screen.size()
    window = SupervisorUI(size.width(), size.height(), nodeID)
    window.show()
    sys.exit(app.exec_())
