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
vector3 = Vector3()

def setup():
    global pub
    rospy.init_node('watchdog', anonymous = True)
    initSerial()
    rospy.Subscriber("vel", Vector3, velCB)
    pub = rospy.Publisher('vel', Vector3, queue_size = 1)

def loop():
    global com, cb, timeStamp, vector3, pub
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if ((cb == True) and (time.time() - timeStamp > HEARTBEAT)):
            rospy.logwarn('Heartbeat timeout')
            vector3.x = 0
            vector3.y = 0
            vector3.z = 0
            pub.publish(vector3)
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
    if (data.x != 0 and data.y != 0 and data.z != 0):
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
