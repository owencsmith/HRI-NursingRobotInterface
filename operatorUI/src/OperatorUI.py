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
rospack = rospkg.RosPack()
designerFile = rospack.get_path('operatorUI')+"/src/OperatorUI.ui"

class OperatorUI(QtWidgets.QMainWindow):
    def __init__(self, width, height):
        super(OperatorUI, self).__init__()
        self.ui = uic.loadUi(designerFile, self)#loads the ui file and adds member names to class
        self.fitToScreen(width, height)
        self.numAnimationSteps = 20
        #self.scaleFactor = 0.4 * self.mapWidth / 1500  # the map was designed for a 1500 pixel square map.This adjusts for the screen size
        self.secondCameraShowing=False
        self.SecondCameraSlideInThread = SecondCameraSlideInThread(self.numAnimationSteps)
        self.SecondCameraSlideInThread.signal.connect(self.SecondCameraSlideInThreadCallback)
        self.scene = QGraphicsScene()
        self.scene.mousePressEvent = self.mapClickEventHandler  # allows for the grid to be clicked
        self.OperatorMap.setMouseTracking(True)
        self.OperatorMap.setScene(self.scene)
        self.RequestCameraBTN.clicked.connect(self.RequestCameraBTNCallback)

    def mapClickEventHandler(self, event):
        pass


    def RequestCameraBTNCallback(self):
        self.SecondCameraSlideInThread.start()

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
        self.ProblemLBL.move(0, robotDescHeight*0.2)
        self.ProblemLBL.resize(robotDescWidth*0.4, robotDescHeight*0.1)
        self.ProblemHereLBL.move(robotDescWidth*0.4, robotDescHeight*0.2)
        self.ProblemHereLBL.resize(robotDescWidth*0.6, robotDescHeight*0.1)
        ######################################################################
        self.RequestCameraBTN.move(0, self.windowHeight*0.95)
        self.RequestCameraBTN.resize(robotDescWidth, self.windowHeight*0.05)
        self.DoneHelpingBTN.move(0, self.windowHeight*0.9)
        self.DoneHelpingBTN.resize(robotDescWidth, self.windowHeight*0.05)
        self.Camera1.move(robotDescWidth, 0)
        self.Camera1.resize(self.windowWidth-robotDescWidth, self.windowHeight)
        self.Camera2.move(robotDescWidth, self.windowHeight)
        self.Camera2.resize(self.windowWidth - robotDescWidth, self.windowHeight/2)

    def SecondCameraSlideInThreadCallback(self, result):
        if(self.secondCameraShowing):
            self.Camera1.resize(self.Camera1.width(), self.windowHeight/2+(result + 1)*(self.windowHeight/2)/self.numAnimationSteps)
            self.Camera2.move(self.Camera2.x(), self.windowHeight/2+(result + 1)*(self.windowHeight/2)/self.numAnimationSteps)
            if(result+1==20):
                self.secondCameraShowing = False
        else:
            self.Camera1.resize(self.Camera1.width(), self.windowHeight - (result + 1) * (self.windowHeight/2)/self.numAnimationSteps)
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
    rospy.init_node('OperatorUI')
    rospy.sleep(.5)
    app = QApplication(sys.argv)
    screen = app.primaryScreen()#gets size of primary screen. This assumes the GUI is being run on the primary screen. If it is not and the secondary screen has a different resolution this will not scale correctly
    size = screen.size()
    window = OperatorUI(size.width(), size.height())
    window.show()
    sys.exit(app.exec_())