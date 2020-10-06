#!/usr/bin/env python3
#import rospy
from PyQt5 import QtCore, QtWidgets, uic
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import sys
designerFile = "SupervisorUI.ui"

class SupervisorUI(QtWidgets.QMainWindow):
    def __init__(self):
        super(SupervisorUI, self).__init__()
        self.ui = uic.loadUi(designerFile, self)#loads the ui file and adds member names to class
        self.AsyncFunctionsThread = AsyncFunctionsThread(self)
        self.AsyncFunctionsThread.signal.connect(self.AsyncFunctionsThreadCallback)
        self.Button1.clicked.connect(self.button1Callback)
        #subscriber for reciecing robot locations on the map

    def AsyncFunctionsThreadCallback(self, result):
        pass

    def keyPressEvent(self, event):#this is overriding qMainWindow
        if event.key() == Qt.Key_M:
            pass

    def button1Callback(self):

        print("Button Pressed")


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