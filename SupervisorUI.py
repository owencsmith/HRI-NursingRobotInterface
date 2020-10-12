#!/usr/bin/env python3
#import rospy
from PyQt5 import QtCore, QtWidgets, uic
from PyQt5.QtWidgets import QApplication, QGraphicsScene, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsTextItem
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, qRgb, QTransform, QFont
from PyQt5.QtCore import QThread, pyqtSignal, QPoint
from PyQt5.QtCore import Qt, QLineF, QRectF
import sys
import json
designerFile = "SupervisorUI.ui"
map = "Maps/Hospital"

class SupervisorUI(QtWidgets.QMainWindow):
    def __init__(self):
        super(SupervisorUI, self).__init__()
        self.ui = uic.loadUi(designerFile, self)#loads the ui file and adds member names to class
        self.AsyncFunctionsThread = AsyncFunctionsThread(self)
        self.AsyncFunctionsThread.signal.connect(self.AsyncFunctionsThreadCallback)
        self.Button1.clicked.connect(self.button1Callback)
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
        self.Graphicswidth = graphicsSize.width()
        self.Graphicsheight = graphicsSize.height()
        self.scene = QGraphicsScene()
        #TODO self.scene.mousePressEvent = self.mapClickEventHandler  # allows for the grid to be clicked
        self.SupervisorMap.setMouseTracking(True)
        #todo self.scene.mouseMoveEvent = self.MouseMovementEvent
        self.SupervisorMap.setScene(self.scene)
        self.loadMap(map)
    def AsyncFunctionsThreadCallback(self, result):
        pass

    def keyPressEvent(self, event):#this is overriding qMainWindow
        if event.key() == Qt.Key_M:
            pass

    def button1Callback(self):

        print("Button Pressed")

    def loadMap(self, mapLocation):
        with open(mapLocation) as file:
            mapObjList = json.load(file)
        print(mapObjList)
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