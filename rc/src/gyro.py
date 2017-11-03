#!/usr/bin/env python
import rospy
import serial
import tf
import math
from geometry_msgs.msg import Vector3

gyro = 0.0
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
        vector3.z = gyro
        pub.publish(vector3)
        gyroInfo = "Gyro = %f" % gyro
        rospy.loginfo(gyroInfo)
        rate.sleep()

def initSerial():
    global com
    try:
        com = serial.Serial('/dev/ttyUSB3', 115200)
        rospy.loginfo('Gyroscope initialized')
    except:
        rospy.logerr('Gyroscope NOT initialized')

def getGyro():
    global com, gyro
    try:
        com.flushInput()
        while (com.read(1) != '\n'):
            pass
        gyro = float(com.read(8)) * math.pi / 180
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
