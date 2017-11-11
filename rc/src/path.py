#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import os
import rospy
from time import sleep
from geometry_msgs.msg import Vector3

PARENT_PATH = os.path.dirname(os.getcwd())

SLEEP = 2
INTERVAL = 1

goalX = []
goalY = []
goalZ = []
idx = 0
vector3 = Vector3()

def setup():
    global SLEEP, pub
    rospy.init_node('path', anonymous = True)
    readPath()
    sleep(SLEEP)
    pub = rospy.Publisher('goal', Vector3, queue_size = 1)

def loop():
    global INTERVAL, vector3, goalX, goalY, goalZ, idx, pub
    rate = rospy.Rate(1 / INTERVAL)
    while not rospy.is_shutdown():
        if (idx < len(goalX)):
            vector3.x = goalX[idx]
            vector3.y = goalY[idx]
            vector3.z = goalZ[idx]
            pub.publish(vector3)
            pathInfo = "No.%d, X = %f, Y = %f, Z = %f" % (idx + 1, vector3.x, vector3.y, vector3.z)
            rospy.loginfo(pathInfo)
            idx += 1
        else:
            rospy.loginfo('Path publish finished')
            rospy.signal_shutdown('Path publish finished')
        rate.sleep()

def readPath():
    global PARENT_PATH, goalX, goalY, goalZ
    try:
        fobj = open(PARENT_PATH + '/bezier.txt', 'r')
        for eachline in fobj:
            buffer = eachline.encode('utf-8').split(' ')
            goalX.append(float(buffer[0]))
            goalY.append(float(buffer[1]))
            goalZ.append(float(buffer[2]))
        fobj.close()
        rospy.loginfo('Path file loaded')
        rospy.loginfo('Point number: %d' % len(goalX))
        rospy.loginfo('Start point: (%f, %f, %f)' % (goalX[0], goalY[0], goalZ[0]))
        rospy.loginfo('End point: (%f, %f, %f)' % (goalX[len(goalX) - 1], goalY[len(goalX) - 1], goalZ[len(goalX) - 1]))
    except:
        rospy.logerr('Path file NOT loaded')
        rospy.signal_shutdown('Path file NOT loaded')

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
    loop()
