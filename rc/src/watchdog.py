#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import serial
import time
from geometry_msgs.msg import Vector3

HEARTBEAT = 2

com = []
cb = False
timeStamp = 0.0

def setup():
    rospy.init_node('watchdog', anonymous = True)
    initSerial()
    rospy.Subscriber("vel", Vector3, velCB)

def loop():
    global com, cb, timeStamp
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if ((cb == True) and (time.time() - timeStamp > HEARTBEAT)):
            rospy.logwarn('Heartbeat timeout')
            for i in range(3):
                try:
                    com[i].write('ST;')
                except:
                    pass
            cb = False
        rate.sleep()

def initSerial():
    global com
    for i in range(3):
        try:
            com.append(serial.Serial('/dev/ttyUSB%d' % i, 19200))
        except:
            pass

def velCB(data):
    global cb, timeStamp
    cb = True
    timeStamp = time.time()

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
    loop()
    for i in range(3):
        try:
            com[i].close()
        except:
            pass
