#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy
import math
from geometry_msgs.msg import Vector3

XY_PRECISION = 0.02
Z_PRECISION = 1 / 180 * math.pi

position = [0] * 3
goal = [0] * 3
dist = [0] * 3
lock1 = True
lock2 = True
vector3 = Vector3()

def setup():
    global pub
    rospy.init_node('goto', anonymous = True)
    rospy.Subscriber("position", Vector3, positionCB)
    rospy.Subscriber("goal", Vector3, goalCB)
    pub = rospy.Publisher('vel', Vector3, queue_size = 1)

def loop():
    global lock1, lock2, vector3, dist, pub
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        getDist()
        vector3.x = limiter(dist[0], -1, 1)
        vector3.y = limiter(dist[1], -1, 1)
        vector3.z = limiter(dist[2], - math.pi / 2, math.pi / 2)
        if (lock1 == False and lock2 == False):
            pub.publish(vector3)
        rate.sleep()

def limiter(value, lower, upper):
    if value < lower:
        return lower
    elif value > upper:
        return upper
    else:
        return value

def getDist():
    global XY_PRECISION, Z_PRECISION, position, goal, dist
    dist[0] = goal[0] - position[0]
    dist[1] = goal[1] - position[1]
    dist[2] = goal[2] - position[2]
    while (dist[2] <= - math.pi):
        dist[2] += 2 * math.pi
    while (dist[2] > math.pi):
        dist[2] -= 2 * math.pi
    if math.sqrt(dist[0] ** 2 + dist[1] ** 2) <= XY_PRECISION:
        dist[0] = 0
        dist[1] = 0
    if math.fabs(dist[2]) <= Z_PRECISION:
        dist[2] = 0

def positionCB(data):
    global lock1, position
    position[0] = data.x
    position[1] = data.y
    position[2] = data.z
    lock1 = False

def goalCB(data):
    global lock2, goal
    goal[0] = data.x
    goal[1] = data.y
    goal[2] = data.z
    lock2 = False

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
    loop()
