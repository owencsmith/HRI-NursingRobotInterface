#!/usr/bin/env python
import time

import rospy
from std_msgs.msg import *
from nav_msgs.msg import OccupancyGrid
from middleman.msg import Robot, RobotArr, TaskMsg, TaskMsgArr
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from gazebo_msgs.msg import ModelState
from middleman.srv import TaskString, TaskStringResponse
from Task import *
from Operator import *
from Supervisor import *
from searchCoordinator import *
import numpy as np

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name
class Tutorial:
    _blockListDict = {
    'a': Block('cardboard_box', 'link'),
        'b': Block('bookshelf_0', 'link'),
        'c': Block('cabinet', 'link'),
        'd': Block('mars_rover', 'link'),
        'e': Block('Dumpster_0', 'link'),
    'f': Block('unit_box_0', 'link'),
        'g': Block('fire_hydrant', 'link'),
    'h': Block('first_2015_trash_can', 'link'),
    }

    def update_gazebo_modelPoints(self):
        try:
            i = 0
            plist = []
            model_coordinates = rospy.ServiceProxy('/gazebo/model_state',ModelState)
            for block in self._blockListDict.itervalues():
                blockName = str(block._name)
                # resp_coordinates = model_coordinates(blockName,block._relative_entity_name)
                resp_coordinates = model_coordinates(blockName, "")
                plist.append(Point(resp_coordinates.pose.position.x,resp_coordinates.pose.position.y,0))
            return plist
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))


    # TIP: add model to gazebo.  then `rostopic echo /gazebo/model_states` to see poses of objects