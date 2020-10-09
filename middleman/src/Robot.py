#!/usr/bin/env python
import rospy

class Robot():
    def __init__(self):
        name = ""
        currentTask = ""
        status = "" # Good (OK), Bad (NOK), Operator Control (OPC)
        dictionaryOfPublishers = []

        #publishers
        statePublisherForOperator = rospy.Publisher('/operator/robotState', String, queue_size=10)
        statePublisherForSupervisor = rospy.Publisher('/supervisor/robotState', String, queue_size=10)


    # idk how to implement this cleanly but we will
    def publish(self, topic, data):
        pass