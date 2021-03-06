#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy
import serial
import math
from geometry_msgs.msg import Vector3

WHEEL_RADIUS = 0.078
SIDE_LENGTH = 1.01
REDUCTION_RATIO = 19

com = []
vel = [0] * 3
gyro = 0.0

def setup():
    rospy.init_node('omni', anonymous = True)
    initSerial()
    initElmo()
    rospy.Subscriber("vel", Vector3, velCB)
    rospy.Subscriber("position", Vector3, gyroCB)

def loop():
    global com, vel
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        omniInfo = "V1 = %d, V2 = %d, V3 = %d" % (vel[0], vel[1], vel[2])
        rospy.loginfo(omniInfo)
        rate.sleep()

def initSerial():
    global com
    for i in range(3):
        try:
            com.append(serial.Serial('/dev/ttyUSB%d' % i, 19200))
            rospy.loginfo('Omni[%d] initialized' % i)
        except:
            rospy.logerr('Omni[%d] NOT initialized' % i)

def initElmo():
    global com
    for i in range(3):
        try:
            com[i].write('MO=0;PM=1;AC=1000000;PM=1000000;UM=2;MO=1;')
        except:
            pass

def velCB(data):
    global com, vel, gyro
    vx = data.x
    vy = data.y
    vz = data.z * SIDE_LENGTH / math.sqrt(3)
    vel = [- vx * math.cos(gyro) - vy * math.sin(gyro) + vz, \
           vx * math.sin(math.pi / 6 + gyro) - vy * math.cos(math.pi / 6 + gyro) + vz, \
           vx * math.sin(math.pi / 6 - gyro) + vy * math.cos(math.pi / 6 - gyro) + vz]
    vel = [int(x / (2 * math.pi * WHEEL_RADIUS) * REDUCTION_RATIO * 2000) for x in vel]
    for i in range(3):
        try:
            com[i].write('JV=%d;BG;' % vel[i])
        except:
            pass

def gyroCB(data):
    global gyro
    gyro = data.z
    """
    quaternion = (data.x, data.y, data.z, data.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    gyro = euler[2]
    """

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
    loop()
    for i in range(3):
        try:
            com[i].write('ST;MO=0;')
            com[i].close()
        except:
            pass
