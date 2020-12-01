#!/usr/bin/env python3
import rospy
import rospkg
from PyQt5 import QtCore, QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QGraphicsScene, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsTextItem, \
    QGraphicsView, QHeaderView, QTableWidgetItem, QListView, QGraphicsLineItem
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, qRgb, QTransform, QFont, QWheelEvent, QStandardItemModel, \
    QImage, QPixmap
from PyQt5.QtCore import QThread, pyqtSignal, QPoint, QPointF
from PyQt5.QtCore import Qt, QLineF, QRectF
import sys
import json
import time
import math
from std_msgs.msg import *
from middleman.msg import Robot, RobotArr
from sensor_msgs.msg import Image, LaserScan
import numpy as np
rospack = rospkg.RosPack()
designerFile = rospack.get_path('operatorUI')+"/src/OperatorUI.ui"
map = rospack.get_path('supervisorUI')+"/src/Maps/Hospital.json"

class OperatorUI(QtWidgets.QMainWindow):
    robotUpdateSignal = pyqtSignal('PyQt_PyObject')
    mainCamUpdateSignal = pyqtSignal('PyQt_PyObject')
    secondaryCamUpdateSignal = pyqtSignal('PyQt_PyObject')
    LaserScanUpdateSignal = pyqtSignal('PyQt_PyObject')

    def __init__(self, width, height, id):
        super(OperatorUI, self).__init__()
        self.ui = uic.loadUi(designerFile, self)#loads the ui file and adds member names to class
        self.fitToScreen(width, height)
        self.numAnimationSteps = 20
        self.scaleFactor = 2 * self.OperatorMap.width() / 1500  # the map was designed for a 1500 pixel square map.This adjusts for the screen size
        self.secondCameraShowing=False
        self.SecondCameraSlideInThread = SecondCameraSlideInThread(self.numAnimationSteps)
        self.SecondCameraSlideInThread.signal.connect(self.SecondCameraSlideInThreadCallback)
        self.RobotShapes = []
        self.RobotNames = []
        self.RoomNames = []
        self.ObstacleList = []
        self.previousMapRotation = 0
        self.numberOfObstacleGroupings = 64
        self.obstacleWarningDistance = 1.0 #in Meters
        self.obstacleMarkerShapes = []
        self.obstacleWarningDistanceSLDR.setMinimum(1)
        self.obstacleWarningDistanceSLDR.setMaximum(20)
        self.obstacleWarningDistanceSLDR.valueChanged.connect(self.updateObstacleDistanceLBL)
        self.obstacleWarningDistanceSLDR.setValue(self.obstacleWarningDistance*2)
        self.robotWarningDistance = 1.0  # in Meters
        self.robotMarkerShapes = []
        self.RobotWarningDistanceSLDR.setMinimum(1)
        self.RobotWarningDistanceSLDR.setMaximum(40)
        self.RobotWarningDistanceSLDR.valueChanged.connect(self.updateRobotDistanceLBL)
        self.RobotWarningDistanceSLDR.setValue(self.robotWarningDistance * 2)
        self.robotUpdateSignal.connect(self.drawRobotCallback)
        self.mainCamUpdateSignal.connect(self.MainCamUpdate)
        self.secondaryCamUpdateSignal.connect(self.SecondaryCamUpdate)
        self.LaserScanUpdateSignal.connect(self.drawObstacleWarnings)
        self.ToggleObstaclesCB.stateChanged.connect(self.obstacleComboBoxChanged)
        self.scene = QGraphicsScene()
        self.scene.mousePressEvent = self.mapClickEventHandler  # allows for the grid to be clicked
        self.augmentedRealityScene = QGraphicsScene()
        self.AugmentedRealityPanel.setScene(self.augmentedRealityScene)
        self.OperatorMap.setMouseTracking(True)
        self.OperatorMap.setScene(self.scene)
        self.RequestCameraBTN.clicked.connect(self.RequestCameraBTNCallback)
        self.ObstacleWarningCB.setChecked(True)
        self.RobotWarningCB.setChecked(True)
        self.black = QColor(qRgb(0, 0, 0))
        self.blue = QColor(qRgb(30, 144, 255))
        self.red = QColor(qRgb(220, 20, 60))
        self.green = QColor(qRgb(0, 255, 127))
        self.Orange = QColor(qRgb(255, 165, 0))
        self.yellow = QColor(qRgb(255, 255, 0))
        self.purple = QColor(qRgb(238, 130, 238))
        self.magenta = QColor(qRgb(255, 0, 255))
        self.white = QColor(qRgb(255, 255, 255))
        self.gray = QColor(qRgb(119, 136, 153))
        self.loadMap(map)
        # Middleman communication stuff
        self.doneHelpingRobotPublisher = rospy.Publisher('/operator/done_helping', String, queue_size=10)
        self.releaseCameraPublisher = rospy.Publisher('/operator/release_help_robot', String, queue_size=10)
        self.DoneHelpingBTN.clicked.connect(self.DoneHelpingBTNCallback)
        self.DoneHelpingBTN.setEnabled(False)
        self.currentRobot = None
        self.helperRobot = None
        self.MainCamSubscriber = rospy.Subscriber('none', Image, self.MainCamSubscriberCallback)#subscriber changes inn new robotcallback
        self.SecondaryCamSubscriber = rospy.Subscriber('none', Image, self.SecondaryCamSubscriberCallback)
        self.LaserScanSubscriber = rospy.Subscriber('none', LaserScan, self.LaserScanCallback)
        self.stateSubscriber = rospy.Subscriber('/operator/robotState', RobotArr, self.robotStateCallback)
        self.newRobotSubscriber = rospy.Subscriber('/' + id + '/new_robot', Robot, self.NewRobotCallback)
        self.robotassignedForExtraCamera = rospy.Subscriber('/' + id +'/robotForExtraCamera', String, self.SecondaryCamRecievedCallback )
        self.requestAnotherRobotForCameraViews= rospy.Publisher('/operator/request_extra_views_from_robot', String, queue_size=10)

        #New operator message for middleman
        self.newOperatorIDPublisher = rospy.Publisher('/operator/new_operator_ui', String, queue_size = 10)

        #heartbeat stuff
        rospy.Timer(rospy.Duration(.5), self.sendHeartBeat)
        self.heartBeatPublisher = rospy.Publisher('/' + id + '/heartbeat', String, queue_size = 10)
        rospy.sleep(1)
        self.newOperatorIDPublisher.publish(id)


    def sendHeartBeat(self, data):
        self.heartBeatPublisher.publish("lubdub")

    def mapClickEventHandler(self, event):
        pass

    def DoneHelpingBTNCallback(self):
        self.doneHelpingRobotPublisher.publish(self.currentRobot.name)
        self.DoneHelpingBTN.setEnabled(False)
        self.NameHereLBL.setText("")
        self.TaskHereLBL.setText("")
        self.RequestCameraBTN.setEnabled(False)
        self.currentRobot =None
        #self.helperRobot = None
        self.MainCamSubscriber.unregister()
        self.SecondaryCamSubscriber.unregister()
        self.LaserScanSubscriber.unregister()
        self.Camera1.clear()
        self.Camera1.setText("Waiting For Video Feed")
        if self.secondCameraShowing:
            self.RequestCameraBTNCallback()
        for item in self.obstacleMarkerShapes:
            self.augmentedRealityScene.removeItem(item)

    def RequestCameraBTNCallback(self):
        if(self.secondCameraShowing):
            # publish the name of the robot that was helping
            self.releaseCameraPublisher.publish(self.helperRobot)
            self.helperRobot = None
            self.SecondaryCamSubscriber.unregister()
            self.Camera2.clear()
            self.Camera2.setText("Waiting For Video Feed")
            self.RequestCameraBTN.setText("Request Camera")
        else:
            self.RequestCameraBTN.setText("Cancel Camera")
            self.requestAnotherRobotForCameraViews.publish(self.currentRobot.name)#if no robot is selected this makes it error out and not change bring in the second screen. a bug disguised as a feature
        self.SecondCameraSlideInThread.start()
        
    def NewRobotCallback(self, data):
        self.DoneHelpingBTN.setDisabled(False)
        self.RequestCameraBTN.setDisabled(False)
        print(data.name)
        self.currentRobot = data
        self.NameHereLBL.setText(self.currentRobot.name)
        self.TaskHereLBL.setText(self.currentRobot.currentTask.taskName)
        self.MainCamSubscriber.unregister()
        self.MainCamSubscriber = rospy.Subscriber("/"+data.name+"/main_cam/color/image_raw", Image, self.MainCamSubscriberCallback)
        self.LaserScanSubscriber.unregister()
        self.LaserScanSubscriber = rospy.Subscriber("/"+data.name+"/base_scan", LaserScan, self.LaserScanCallback)

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
                label.setTransformOriginPoint(QPoint(item["length"] / 2, item["width"]/2))
                label.setRotation(item["rotation"])
                self.RoomNames.append(label)
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
        self.OperatorMap.scale(self.scaleFactor, self.scaleFactor)

    def robotStateCallback(self, message):
        #print("MSG received")
        self.robotUpdateSignal.emit(message)

    def drawRobotCallback(self, result):
        for item in self.RobotShapes:
            self.scene.removeItem(item)
        self.RobotShapes.clear()
        self.RobotNames.clear()
        for item in self.robotMarkerShapes:
            self.augmentedRealityScene.removeItem(item)
        self.robotMarkerShapes.clear()
        frameRotation = 0
        obSize = 70
        controlledRobot = None
        for item in result.robots:
            shape = QGraphicsEllipseItem(int(item.pose.pose.pose.position.x*100 - obSize / 2), -int(item.pose.pose.pose.position.y*100 + obSize / 2), obSize, obSize)
            shape.setPen(QPen(self.black))
            color = self.yellow
            if self.currentRobot is not None:
                if item.name == self.currentRobot.name:
                    color = self.blue
                    controlledRobot = item
            if self.helperRobot is not None:
                if self.helperRobot == item.name:
                    color = self.purple
            shape.setBrush(QBrush(color, Qt.SolidPattern))
            self.scene.addItem(shape)
            self.RobotShapes.append(shape)
            self.RobotNames.append(item.name)
            quat = item.pose.pose.pose.orientation
            siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
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
            if self.currentRobot is not None:
                if item.name == self.currentRobot.name:
                    self.OperatorMap.centerOn(int(item.pose.pose.pose.position.x * 100),
                                              -int(item.pose.pose.pose.position.y * 100))
                    deltaRotate = math.degrees(yaw)-90-self.previousMapRotation
                    self.previousMapRotation = math.degrees(yaw)-90
                    self.OperatorMap.rotate(deltaRotate)
                    frameRotation = math.degrees(yaw)-90
            else:
                self.OperatorMap.rotate(-self.previousMapRotation)
                self.previousMapRotation = 0
                frameRotation = 0

        for item in result.robots:
            color = self.yellow
            if self.currentRobot is not None:
                if item.name == self.currentRobot.name:
                    color = self.blue
            if self.helperRobot is not None:
                if self.helperRobot == item.name:
                    color = self.purple
            label = QGraphicsTextItem(item.name)
            label.setX(int(item.pose.pose.pose.position.x * 100 + obSize * 1.1*math.cos(math.radians(-frameRotation-45))))
            label.setY(int(-item.pose.pose.pose.position.y * 100 + obSize * 1.1*math.sin(math.radians(-frameRotation-45))))
            label.setRotation(-frameRotation)
            font = QFont("Bavaria")
            font.setPointSize(18)
            font.setWeight(QFont.Bold)
            label.setDefaultTextColor(color)
            label.setFont(font)
            self.scene.addItem(label)
            self.RobotShapes.append(label)
        for item in self.RoomNames:
            item.setRotation(-frameRotation)
        if self.RobotWarningCB.isChecked():
            for item in result.robots:#draw arrows to other bots
                if controlledRobot is not None:
                    if item.name is not controlledRobot.name:
                        deltaX = controlledRobot.pose.pose.pose.position.x - item.pose.pose.pose.position.x
                        deltaY = controlledRobot.pose.pose.pose.position.y - item.pose.pose.pose.position.y
                        robotDistance = math.sqrt(
                            math.pow(deltaX, 2)+
                            math.pow(deltaY, 2))
                        if robotDistance<self.robotWarningDistance:
                            color = self.yellow
                            if self.helperRobot is not None:
                                if self.helperRobot == item.name:
                                    color = self.purple
                            quat = controlledRobot.pose.pose.pose.orientation
                            siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
                            cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
                            yaw = math.atan2(siny_cosp, cosy_cosp)
                            difAngle = math.atan2(deltaY, deltaX)
                            totalAngle = yaw-difAngle+math.pi/2
                            startArrowX = int(self.AugmentedRealityPanel.height()*math.cos(totalAngle)*0.45)
                            startArrowY = int(self.AugmentedRealityPanel.height()*math.sin(totalAngle)*0.45)
                            endArrowX = int(self.AugmentedRealityPanel.height()*math.cos(totalAngle)*0.49)
                            endArrowY = int(self.AugmentedRealityPanel.height()*math.sin(totalAngle)*0.49)
                            line = QGraphicsLineItem(startArrowX, startArrowY, endArrowX,endArrowY)
                            line.setPen(QPen(color, 10))
                            self.augmentedRealityScene.addItem(line)
                            self.robotMarkerShapes.append(line)
                            line = QGraphicsLineItem(endArrowX,
                                                     endArrowY,
                                                     endArrowX - math.cos(3.14 / 4 + totalAngle) * self.AugmentedRealityPanel.height()*0.02,
                                                     endArrowY - math.sin(3.14 / 4 + totalAngle) * self.AugmentedRealityPanel.height()*0.02)

                            line.setPen(QPen(color, 10))
                            self.augmentedRealityScene.addItem(line)
                            self.robotMarkerShapes.append(line)
                            line = QGraphicsLineItem(endArrowX,
                                                     endArrowY,
                                                     endArrowX - math.cos(3.14 / 4 - totalAngle) * self.AugmentedRealityPanel.height()*0.02,
                                                     endArrowY + math.sin(3.14 / 4 - totalAngle) * self.AugmentedRealityPanel.height()*0.02)

                            line.setPen(QPen(color, 10))
                            self.augmentedRealityScene.addItem(line)
                            self.robotMarkerShapes.append(line)

    def MainCamSubscriberCallback(self, result):
        self.mainCamUpdateSignal.emit(result)

    def MainCamUpdate(self, result):
        image = QImage(result.data, result.width, result.height, result.step, QImage.Format_RGB888)
        image = image.scaled(self.Camera1.width(), self.Camera1.height(), Qt.KeepAspectRatio)
        pixmap = QPixmap.fromImage(image)
        self.Camera1.setPixmap(pixmap)

    def SecondaryCamSubscriberCallback(self, result):
        self.secondaryCamUpdateSignal.emit(result)

    def SecondaryCamUpdate(self, result):
        image = QImage(result.data, result.width, result.height, result.step, QImage.Format_RGB888)
        image = image.scaled(self.Camera2.width(), self.Camera2.height(), Qt.KeepAspectRatio)
        pixmap = QPixmap.fromImage(image)
        self.Camera2.setPixmap(pixmap)

    def SecondaryCamRecievedCallback(self, result):
        print("got second cam")
        print("result")
        self.helperRobot = result.data
        self.SecondaryCamSubscriber.unregister()
        self.SecondaryCamSubscriber = rospy.Subscriber("/" + result.data + "/main_cam/color/image_raw", Image,
                                                  self.SecondaryCamSubscriberCallback)

    def LaserScanCallback(self, result):
        self.LaserScanUpdateSignal.emit(result)

    def drawObstacleWarnings(self, result):
        for item in self.obstacleMarkerShapes:
            self.augmentedRealityScene.removeItem(item)
        self.obstacleMarkerShapes.clear()
        shape0 = QGraphicsEllipseItem(-self.AugmentedRealityPanel.height() / 2,
                                      -self.AugmentedRealityPanel.height() / 2, self.AugmentedRealityPanel.height(),
                                      self.AugmentedRealityPanel.height())
        shape0.setPen(QPen(self.black))
        shape0.setOpacity(0.01)
        self.augmentedRealityScene.addItem(shape0)
        self.obstacleMarkerShapes.append(shape0)
        if (self.ObstacleWarningCB.isChecked()):
            markersToShow = np.zeros(self.numberOfObstacleGroupings)
            startAngle = result.angle_min
            angleIncrement = result.angle_increment
            AngleCutOff = 2 * np.pi / self.numberOfObstacleGroupings
            offset = np.pi/2
            for point in range(0, len(result.ranges)):
                if result.ranges[point]<=self.obstacleWarningDistance:
                    angle = startAngle + angleIncrement*point
                    directionSector = int(angle/AngleCutOff)
                    markersToShow[directionSector]=1
            for marker in range(0, self.numberOfObstacleGroupings):
                if(markersToShow[marker]):
                    shape = QGraphicsLineItem(-self.AugmentedRealityPanel.height()*0.98/2*math.cos(marker*AngleCutOff+offset),-self.AugmentedRealityPanel.height()*0.98/2*math.sin(marker*AngleCutOff+offset), -self.AugmentedRealityPanel.height()*0.98/2*math.cos((marker+1)*AngleCutOff+offset),-self.AugmentedRealityPanel.height()*0.98/2*math.sin((marker+1)*AngleCutOff+offset))
                    shape.setPen(QPen(self.red, 5))
                    self.augmentedRealityScene.addItem(shape)
                    self.obstacleMarkerShapes.append(shape)
        self.AugmentedRealityPanel.centerOn(QPoint(0, 0))

    def updateObstacleDistanceLBL(self):
        self.obstacleWarningDistance = self.obstacleWarningDistanceSLDR.value()/2
        self.ObstacleWarningDistanceLBL.setText(str(self.obstacleWarningDistance) + " M")

    def updateRobotDistanceLBL(self):
        self.robotWarningDistance = self.RobotWarningDistanceSLDR.value() / 2
        self.RobotWarningDistanceLBL.setText(str(self.robotWarningDistance) + " M")

    def obstacleComboBoxChanged(self):
        if self.ToggleObstaclesCB.isChecked():
            for item in self.ObstacleList:
                self.scene.addItem(item)
        else:
            for item in self.ObstacleList:
                self.scene.removeItem(item)

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
        #other objects in the UI get moved and resized here
        robotDescHeight = self.windowHeight*0.3
        robotDescWidth = self.windowWidth*0.2
        self.OperatorMap.move(0, 0)
        self.OperatorMap.resize(robotDescWidth, self.windowHeight*0.6)
        self.OperatorMap.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.OperatorMap.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.ToggleObstaclesCB.move(self.windowWidth * 0.01, self.windowHeight*0.55)
        self.ToggleObstaclesCB.resize(self.windowWidth*0.2, self.windowHeight*0.05)
        ######## Description frame
        self.RobotDescriptionFrame.move(0, self.windowHeight*0.9-robotDescHeight)
        self.RobotDescriptionFrame.resize(robotDescWidth, robotDescHeight)
        self.robotNameLBL.move(0, 0)
        self.robotNameLBL.resize(robotDescWidth*0.4, robotDescHeight*0.1)
        self.NameHereLBL.move(robotDescWidth*0.4, 0)
        self.NameHereLBL.resize(robotDescWidth*0.6, robotDescHeight*0.1)
        self.TaskLBL.move(0, robotDescHeight*0.1)
        self.TaskLBL.resize(robotDescWidth*0.4, robotDescHeight*0.1)
        self.TaskHereLBL.move(robotDescWidth*0.4, robotDescHeight*0.1)
        self.TaskHereLBL.resize(robotDescWidth*0.6, robotDescHeight*0.1)
        self.obstacleWarningDistanceLBL.move(0, robotDescHeight*0.2)
        self.obstacleWarningDistanceLBL.resize(robotDescWidth, robotDescHeight*0.1)
        self.obstacleWarningDistanceSLDR.move(robotDescWidth*0.01, robotDescHeight*0.3)
        self.obstacleWarningDistanceSLDR.resize(robotDescWidth*0.4, robotDescHeight*0.1)
        self.ObstacleWarningDistanceLBL.move(robotDescWidth*0.42, robotDescHeight*0.3)
        self.ObstacleWarningDistanceLBL.resize(robotDescWidth*0.2, robotDescHeight*0.1)
        self.ObstacleWarningCB.move(robotDescWidth*0.63, robotDescHeight*0.3)
        self.ObstacleWarningCB.resize(robotDescWidth*0.36, robotDescHeight*0.1)

        self.RobotWarningDistanceTitleLBL.move(0, robotDescHeight * 0.4)
        self.RobotWarningDistanceTitleLBL.resize(robotDescWidth, robotDescHeight * 0.1)
        self.RobotWarningDistanceSLDR.move(robotDescWidth * 0.01, robotDescHeight * 0.5)
        self.RobotWarningDistanceSLDR.resize(robotDescWidth * 0.4, robotDescHeight * 0.1)
        self.RobotWarningDistanceLBL.move(robotDescWidth * 0.42, robotDescHeight * 0.5)
        self.RobotWarningDistanceLBL.resize(robotDescWidth * 0.2, robotDescHeight * 0.1)
        self.RobotWarningCB.move(robotDescWidth * 0.63, robotDescHeight * 0.5)
        self.RobotWarningCB.resize(robotDescWidth * 0.36, robotDescHeight * 0.1)

        ######################################################################
        self.RequestCameraBTN.move(0, self.windowHeight*0.95)
        self.RequestCameraBTN.resize(robotDescWidth, self.windowHeight*0.05)
        self.DoneHelpingBTN.move(0, self.windowHeight*0.9)
        self.DoneHelpingBTN.resize(robotDescWidth, self.windowHeight*0.05)
        self.Camera1.move(robotDescWidth, 0)
        self.Camera1.resize(self.windowWidth-robotDescWidth, self.windowHeight)
        self.AugmentedRealityPanel.move(robotDescWidth, 0)
        self.AugmentedRealityPanel.resize(self.windowWidth-robotDescWidth, self.windowHeight)
        self.Camera2.move(robotDescWidth, self.windowHeight)
        self.Camera2.resize(self.windowWidth - robotDescWidth, self.windowHeight/2)

    def SecondCameraSlideInThreadCallback(self, result):
        if(self.secondCameraShowing):
            self.Camera1.resize(self.Camera1.width(), self.windowHeight/2+(result + 1)*(self.windowHeight/2)/self.numAnimationSteps)
            self.AugmentedRealityPanel.resize(self.Camera1.width(), self.windowHeight/2+(result + 1)*(self.windowHeight/2)/self.numAnimationSteps)
            self.Camera2.move(self.Camera2.x(), self.windowHeight/2+(result + 1)*(self.windowHeight/2)/self.numAnimationSteps)
            if(result+1==20):
                self.secondCameraShowing = False
        else:
            self.Camera1.resize(self.Camera1.width(), self.windowHeight - (result + 1) * (self.windowHeight/2)/self.numAnimationSteps)
            self.AugmentedRealityPanel.resize(self.Camera1.width(), self.windowHeight - (result + 1) * (self.windowHeight/2)/self.numAnimationSteps)
            self.Camera2.move(self.Camera2.x(),self.windowHeight - (result + 1) * (self.windowHeight/2)/self.numAnimationSteps)
            if (result + 1 == 20):
                self.secondCameraShowing = True

class SecondCameraSlideInThread(QThread):
    signal = pyqtSignal('PyQt_PyObject')

    def __init__(self, numSteps):
        QThread.__init__(self)
        self.numSec = 0.2
        self.numSteps = numSteps

    def run(self):
        for x in range(0, self.numSteps):
            time.sleep(self.numSec / self.numSteps)
            self.signal.emit(x)

if __name__ == '__main__':
    nodeID = "operatorUI_" + str(int(time.time()))
    rospy.init_node(nodeID)
    rospy.sleep(.5)
    app = QApplication(sys.argv)
    screen = app.primaryScreen()#gets size of primary screen. This assumes the GUI is being run on the primary screen. If it is not and the secondary screen has a different resolution this will not scale correctly
    size = screen.size()
    window = OperatorUI(size.width(), size.height(), nodeID)
    window.show()
    sys.exit(app.exec_())