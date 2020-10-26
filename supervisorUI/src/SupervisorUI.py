#!/usr/bin/env python3
import rospy
from PyQt5 import QtCore, QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QGraphicsScene, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsTextItem, \
    QGraphicsView, QHeaderView, QTableWidgetItem
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, qRgb, QTransform, QFont, QWheelEvent
from PyQt5.QtCore import QThread, pyqtSignal, QPoint, QPointF
from PyQt5.QtCore import Qt, QLineF, QRectF
import sys
import json
import time
from std_msgs.msg import *
from supervisorUI.msg import Robot, RobotArr, TaskMsg, TaskMsgArr
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
designerFile = "SupervisorUI.ui" #TODO make this visible inside rosrun
map = "Maps/Hospital" #TODO make this visible inside rosrun

class SupervisorUI(QtWidgets.QMainWindow):
    def __init__(self, width, height):
        super(SupervisorUI, self).__init__()
        self.ui = uic.loadUi(designerFile, self)#loads the ui file and adds member names to class
        self.slideInMenuWidth = 0 # these all get set by fitToScreen method
        self.windowHeight = 0
        self.windowWidth = 0
        self.mapWidth = 0
        self.numAnimationSteps = 20
        self.fitToScreen(width, height)
        self.statePublisher = rospy.Subscriber('/supervisor/robotState', RobotArr, self.robotStateCallback)
        self.AsyncFunctionsThread = AsyncFunctionsThread(self)
        self.AsyncFunctionsThread.signal.connect(self.AsyncFunctionsThreadCallback)
        self.SideMenuThread = SideMenuThread()
        self.SideMenuThread.signal.connect(self.SideMenuThreadCallback)
        self.CreateTaskButton.clicked.connect(self.ToggleSideMenu)
        #subscriber for reciecing robot locations on the map
        graphicsSize = self.SupervisorMap.size()
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
        #TODO self.scene.mousePressEvent = self.mapClickEventHandler  # allows for the grid to be clicked
        self.SupervisorMap.setMouseTracking(True)
        #todo self.scene.mouseMoveEvent = self.MouseMovementEvent
        self.SupervisorMap.wheelEvent = self.wheelEvent
        self.SupervisorMap.setScene(self.scene)
        self.loadMap(map)
        self.SideMenuShowing = False
        self.SetUpTable()

    def ToggleSideMenu(self):
        self.SideMenuThread.start()

    def AsyncFunctionsThreadCallback(self, result):
        pass

    def SideMenuThreadCallback(self, result):
        if(self.SideMenuShowing):
            self.CreateTaskButton.setText("Create Task")
            self.SupervisorMap.resize((self.mapWidth-self.slideInMenuWidth)+(result+1)*self.slideInMenuWidth/self.numAnimationSteps, self.windowHeight)
            self.SideMenu.move(self.windowWidth-self.slideInMenuWidth+(result+1)*self.slideInMenuWidth/self.numAnimationSteps, 0)
            if(result+1==20):
                self.SideMenuShowing = False
        else:
            self.CreateTaskButton.setText("Cancel")
            self.SupervisorMap.resize(self.mapWidth - (result + 1) * self.slideInMenuWidth/self.numAnimationSteps, self.windowHeight)
            self.SideMenu.move(self.windowWidth - (result + 1) * self.slideInMenuWidth/self.numAnimationSteps, 0)
            if (result + 1 == 20):
                self.SideMenuShowing = True

    def keyPressEvent(self, event):#this is overriding qMainWindow
        if event.key() == Qt.Key_M:
            pass

    def SetUpTable(self):
        self.RobotListTable.setColumnCount(2)
        self.RobotListTable.setHorizontalHeaderLabels(('Robot ID', 'Task'))
        self.RobotListTable.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)

    def loadMap(self, mapLocation):
        #These shapes just help make the viewport more centered since it autocenters around the bounding box of the map objects
        shape0 = QGraphicsRectItem(-1500, -1000, 1, 1)
        shape0.setPen(self.white)
        self.scene.addItem(shape0)
        shape1 = QGraphicsRectItem(1500, 1500, 1, 1)
        shape1.setPen(self.white)
        self.scene.addItem(shape1)
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
        self.RobotListTable.setRowCount(0)#clear table
        for item in message.robots:
            self.RobotListTable.insertRow(self.RobotListTable.rowCount())
            self.RobotListTable.setItem(self.RobotListTable.rowCount()-1, 0, QTableWidgetItem(item.name))
            self.RobotListTable.setItem(self.RobotListTable.rowCount()-1, 1, QTableWidgetItem(item.currentTaskName))


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
        self.RobotListTable.resize(robotListWidth, self.windowHeight)
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

        self.SelectRobot.move(self.slideInMenuWidth*0.05, self.windowHeight*0.11)
        self.SelectRobot.resize(self.slideInMenuWidth * 0.45, self.windowHeight * 0.04)
        self.AssignedRobotTXTLabel.move(self.slideInMenuWidth*0.52, self.windowHeight*0.11)
        self.AssignedRobotTXTLabel.resize(self.slideInMenuWidth*0.45, self.windowHeight*0.02)
        self.AssignedRobotLabel.move(self.slideInMenuWidth*0.52, self.windowHeight*0.13)
        self.AssignedRobotLabel.resize(self.slideInMenuWidth*0.45, self.windowHeight*0.02)

        self.SelectLocationBTN.move(self.slideInMenuWidth*0.05, self.windowHeight*0.16)
        self.SelectLocationBTN.resize(self.slideInMenuWidth * 0.45, self.windowHeight * 0.04)
        self.XLocLabel.move(self.slideInMenuWidth*0.52, self.windowHeight*0.16)
        self.XLocLabel.resize(self.slideInMenuWidth*0.45, self.windowHeight*0.02)
        self.YLocLabel.move(self.slideInMenuWidth*0.52, self.windowHeight*0.18)
        self.YLocLabel.resize(self.slideInMenuWidth*0.45, self.windowHeight*0.02)





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

class AsyncFunctionsThread(QThread):
    signal = pyqtSignal('PyQt_PyObject')
    def __init__(self, gui):
        QThread.__init__(self)
        self.gui = gui

    def run(self):
        #does something

        #errors occurs, signal main app
        self.signal.emit("Error")

if __name__ == '__main__':
    rospy.init_node('SupervisorUI')
    rospy.sleep(.5)
    app = QApplication(sys.argv)
    screen = app.primaryScreen()
    size = screen.size()
    window = SupervisorUI(size.width(), size.height())
    window.show()
    sys.exit(app.exec_())