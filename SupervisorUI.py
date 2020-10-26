#!/usr/bin/env python3
#import rospy
from PyQt5 import QtCore, QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QGraphicsScene, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsTextItem, QGraphicsView
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, qRgb, QTransform, QFont, QWheelEvent
from PyQt5.QtCore import QThread, pyqtSignal, QPoint, QPointF
from PyQt5.QtCore import Qt, QLineF, QRectF
import sys
import json
import time
designerFile = "SupervisorUI.ui"
map = "Maps/Hospital"

class SupervisorUI(QtWidgets.QMainWindow):
    def __init__(self):
        super(SupervisorUI, self).__init__()
        self.ui = uic.loadUi(designerFile, self)#loads the ui file and adds member names to class
        self.setFixedSize(1600, 1080)
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
        self.scaleFactor = 0.4
        self.scene = QGraphicsScene()
        #TODO self.scene.mousePressEvent = self.mapClickEventHandler  # allows for the grid to be clicked
        self.SupervisorMap.setMouseTracking(True)
        #todo self.scene.mouseMoveEvent = self.MouseMovementEvent
        self.SupervisorMap.wheelEvent = self.wheelEvent
        self.SupervisorMap.setScene(self.scene)
        self.SupervisorMap.resize(1400, 1080)
        self.SideMenu.move(1600, 0)
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
            self.SupervisorMap.resize(1200+(result+1)*10, 1080)
            self.SideMenu.move(1400+(result+1)*10, 0)
            if(result+1==20):
                self.SideMenuShowing = False
        else:
            self.CreateTaskButton.setText("Cancel")
            self.SupervisorMap.resize(1400 - (result + 1) * 10, 1080)
            self.SideMenu.move(1600 - (result + 1) * 10, 0)
            if (result + 1 == 20):
                self.SideMenuShowing = True

    def keyPressEvent(self, event):#this is overriding qMainWindow
        if event.key() == Qt.Key_M:
            pass

    def SetUpTable(self):
        self.RobotListTable.setColumnCount(2)
        self.RobotListTable.setHorizontalHeaderLabels(('Robot ID', 'Task'))

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
                self.scaleFactor += 0.01
        elif event.angleDelta().y()<0:
            if (self.scaleFactor > 0.4):
                self.scaleFactor -= 0.01
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
    #rospy.init_node('SupervisorUI')
    #rospy.sleep(.5)
    app = QApplication(sys.argv)
    window = SupervisorUI()
    window.show()
    sys.exit(app.exec_())