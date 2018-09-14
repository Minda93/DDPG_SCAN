#!/usr/bin/env python3
# -*- coding: utf-8 -*-+

import rospy


# rostopic msg
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

class NodeHandle(object):
    def __init__(self):
        self.__start = 1
        self.__scan = None
        self.__Angle = 20
        
        # param
        self.__finalAng = 90
        self.__maxVel = 10
        self.__minVel = 20

        self.__loadParam = False

        # publish
        self.pub_cmdvel = rospy.Publisher('motion/cmd_vel',Twist, queue_size = 1)

        # Subscriber
        rospy.Subscriber("/vision/BlackRealDis", Int32MultiArray,self.Sub_Scan)
    
    def Sub_Scan(self,msg):
        self.__scan = msg.data
    


    @property
    def start(self):
        return self.__start

    @start.setter
    def start(self, value):
        self.__start = value

    @property
    def scan(self):
        return self.__scan

    @scan.setter
    def scan(self, value):
        self.__scan = value

    @property
    def Angle(self):
        return self.__Angle

    @Angle.setter
    def Angle(self, value):
        self.__Angle = value

    @property
    def finalAng(self):
        return self.__finalAng

    @finalAng.setter
    def finalAng(self, value):
        self.__finalAng = value

    @property
    def maxVel(self):
        return self.__maxVel

    @maxVel.setter
    def maxVel(self, value):
        self.__maxVel = value
    @property
    def minVel(self):
        return self.__minVel

    @minVel.setter
    def minVel(self, value):
        self.__minVel = value
    