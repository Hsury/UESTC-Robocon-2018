#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy
import serial
import math
from geometry_msgs.msg import Vector3

gyro = [0] * 3
vector3 = Vector3()

def setup():
    global pub
    rospy.init_node('gyro', anonymous = True)
    initSerial()
    pub = rospy.Publisher('gyro', Vector3, queue_size = 1)

def loop():
    global com, vector3, gyro, pub
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        getGyro()
        vector3.x = gyro[0]
        vector3.y = gyro[1]
        vector3.z = gyro[2]
        pub.publish(vector3)
        gyroInfo = "X = %f, Y = %f, Z = %f" % (gyro[0], gyro[1], gyro[2])
        rospy.loginfo(gyroInfo)
        rate.sleep()

def initSerial():
    global com
    try:
        com = serial.Serial('/dev/ttyUSB3', 921600)
        rospy.loginfo('Gyroscope initialized')
    except:
        rospy.logerr('Gyroscope NOT initialized')

def getGyro():
    global com, gyro
    try:
        com.flushInput()
        while (com.read(1) != '\n'):
            pass
        buffer = ''
        while True:
            buffer += com.read(1)
            if (buffer[-1] == '\r'):
                buffer = buffer[:-1].split(' ')
                break
        gyro[0] = int(buffer[1]) / 1E4
        gyro[1] = int(buffer[2]) / 1E4
        gyro[2] = float(buffer[0]) * math.pi / 180
    except:
        pass

"""
def euler2quaternion():
    global gyro, pose
    quaternion = tf.transformations.quaternion_from_euler(0, 0, gyro * math.pi / 180)
    pose.x = quaternion[0]
    pose.y = quaternion[1]
    pose.z = quaternion[2]
    pose.w = quaternion[3]
"""

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
    loop()
    try:
        com.close()
    except:
        pass
