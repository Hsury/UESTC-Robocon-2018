#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy
from geometry_msgs.msg import Vector3

RESET = True

offset = [0] * 3
gyroData = [0] * 3
lock = True
vector3 = Vector3()

def setup():
    global pub
    rospy.init_node('position', anonymous = True)
    rospy.Subscriber("swift", Vector3, swiftCB)
    rospy.Subscriber("gyro", Vector3, gyroCB)
    pub = rospy.Publisher('position', Vector3, queue_size = 1)

def loop():
    global vector3, pub
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        merge()
        pub.publish(vector3)
        positionInfo = "X = %f, Y = %f, Z = %f" % (vector3.x, vector3.y, vector3.z)
        rospy.loginfo(positionInfo)
        rate.sleep()

def merge():
    global RESET, lock, vector3, offset, gyroData
    vector3.x = offset[0] + gyroData[0]
    vector3.y = offset[1] + gyroData[1]
    vector3.z = offset[2] + gyroData[2]
    if RESET == True and lock == False:
        offset[0] = - vector3.x
        offset[1] = - vector3.y
        offset[2] = - vector3.z
        RESET = False

def swiftCB(data):
    global vector3, offset
    offset[0] = data.x - vector3.x
    offset[1] = data.y - vector3.y
    offset[2] = data.z - vector3.z

def gyroCB(data):
    global lock, gyroData
    gyroData[0] = data.x
    gyroData[1] = data.y
    gyroData[2] = data.z
    lock = False

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
    loop()
