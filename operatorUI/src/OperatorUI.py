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
        self.RequestCameraBTN.move(0, self.windowHeight-100)
        self.RequestCameraBTN.resize(self.windowWidth*0.08, 100)
        self.DoneHelpingBTN.move(0, self.windowHeight-200)
        self.DoneHelpingBTN.resize(self.windowWidth*0.08, 100)


if __name__ == '__main__':
    rospy.init_node('OperatorUI')
    rospy.sleep(.5)
    app = QApplication(sys.argv)
    screen = app.primaryScreen()#gets size of primary screen. This assumes the GUI is being run on the primary screen. If it is not and the secondary screen has a different resolution this will not scale correctly
    size = screen.size()
    window = OperatorUI(size.width(), size.height())
    window.show()
    sys.exit(app.exec_())