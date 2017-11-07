#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy
from geometry_msgs.msg import Vector3

gyroData = [0] * 3
vector3 = Vector3()

def setup():
    global pub
    rospy.init_node('position', anonymous = True)
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
    global vector3, gyroData
    vector3.x = gyroData[0] + 7
    vector3.y = gyroData[1] + 7
    vector3.z = gyroData[2]

def gyroCB(data):
    global gyroData
    gyroData[0] = data.x
    gyroData[1] = data.y
    gyroData[2] = data.z

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
    loop()
