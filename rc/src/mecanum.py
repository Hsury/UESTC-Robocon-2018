#!/usr/bin/env python
import rospy
import serial
import tf
import math
from geometry_msgs.msg import Vector3

WHEEL_RADIUS = 0.078
SIDE_LENGTH = 1.01
REDUCTION_RATIO = 19

com = []
vel = [0, 0, 0, 0]
gyro = 0.0

def setup():
    rospy.init_node('mecanum', anonymous = True)
    initSerial()
    initElmo()
    rospy.Subscriber("vel", Vector3, velCB)
    rospy.Subscriber("gyro", Vector3, gyroCB)

def loop():
    global com, vel
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        mecanumInfo = "V1 = %d, V2 = %d, V3 = %d, V4 = %d" % (vel[0], vel[1], vel[2], vel[3])
        rospy.loginfo(mecanumInfo)
        rate.sleep()

def initSerial():
    global com
    for i in range(4):
        try:
            com.append(serial.Serial('/dev/ttyUSB%d' % i, 19200))
            rospy.loginfo('Mecanum[%d] initialized' % i)
        except:
            rospy.logerr('Mecanum[%d] NOT initialized' % i)

def initElmo():
    global com
    for i in range(4):
        try:
            com[i].write('MO=0;PM=1;AC=1000000;PM=1000000;UM=2;MO=1;')
        except:
            pass

def velCB(data):
    global com, vel, gyro
    vx = data.x
    vy = data.y
    vz = data.z * SIDE_LENGTH * math.sqrt(2) / 2
    vel = [(vx * math.sin(math.pi / 4 - gyro) + vy * math.cos(math.pi / 4 - gyro) + vz) * math.sqrt(2), \
           (- vx * math.cos(math.pi / 4 - gyro) + vy * math.sin(math.pi / 4 - gyro) + vz) * math.sqrt(2), \
           - vel[0], \
           - vel[1]]
    vel = [int(x / (2 * math.pi * WHEEL_RADIUS) * REDUCTION_RATIO * 2000) for x in vel]
    for i in range(4):
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
    for i in range(4):
        try:
            com[i].write('ST;MO=0;')
            com[i].close()
        except:
            pass
