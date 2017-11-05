#!/usr/bin/env python
import rospy
import threading
from evdev import InputDevice, categorize, ecodes
from geometry_msgs.msg import Vector3

x = 0.0
y = 0.0
gyro = 0.0
vector3 = Vector3()

def setup():
    global pub
    rospy.init_node('mouse', anonymous = True)
    initMouse()
    rospy.Subscriber("gyro", Vector3, gyroCB)
    pub = rospy.Publisher('mouse', Vector3, queue_size = 1)

def loop():
    global vector3, x, y, pub
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        vector3.x = x
        vector3.y = y
        pub.publish(vector3)
        mouseInfo = "X = %f, Y = %f" % (x, y)
        rospy.loginfo(mouseInfo)
        rate.sleep()

def initMouse():
    global dev
    try:
        dev = InputDevice('/dev/input/event2')
        thd = threading.Thread(target = mouseEventLoop)
        thd.setDaemon(True)
        thd.start()
        rospy.loginfo('Mouse initialized')
    except:
        rospy.logerr('Mouse NOT initialized')

def mouseEventLoop():
    global dev, x, y, gyro
    for event in dev.read_loop():
        if event.type == ecodes.EV_REL:
            if event.code == ecodes.REL_X:
                x += event.value * math.cos(gyro)
                y += event.value * math.sin(gyro)
            if event.code == ecodes.REL_Y:
                x += event.value * math.sin(gyro)
                y -= event.value * math.cos(gyro)

def gyroCB(data):
    global gyro
    gyro = data.z

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
    loop()
