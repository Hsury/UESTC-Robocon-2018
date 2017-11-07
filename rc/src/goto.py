#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy
import math
from geometry_msgs.msg import Vector3

position = [0] * 3
goal = [0] * 3
dist = [0] * 3
vector3 = Vector3()

def setup():
    global pub
    rospy.init_node('goto', anonymous = True)
    rospy.Subscriber("position", Vector3, positionCB)
    rospy.Subscriber("goal", Vector3, goalCB)
    pub = rospy.Publisher('vel', Vector3, queue_size = 1)

def loop():
    global vector3, dist, pub
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        plan()
        vector3.x = dist[0] / 20
        vector3.y = dist[1] / 20
        vector3.z = dist[2] / 20
        pub.publish(vector3)
        rate.sleep()

def plan():
    global position, goal, dist
    dist[0] = goal[0] - position[0]
    dist[1] = goal[1] - position[1]
    dist[2] = goal[2] - position[2]
    while (dist[2] <= - math.pi):
        dist[2] += 2 * math.pi
    while (dist[2] > math.pi):
        dist[2] -= 2 * math.pi

def positionCB(data):
    global position
    position[0] = data.x
    position[1] = data.y
    position[2] = data.z

def goalCB(data):
    global goal
    goal[0] = data.x
    goal[1] = data.y
    goal[2] = data.z

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
    loop()
