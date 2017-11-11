#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import sys
import rospy
import rospkg
import math
import threading
import numpy as np
from geometry_msgs.msg import Vector3
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

MAP_SIZE = 800
ARROW_SIZE = 15
MARGIN = [1.5, 0.5]

schePts = np.zeros((1, 3))
realPts = np.zeros((1, 3))
lock = True
stateMachine = 0
vector3 = Vector3()

def setup():
    global rospack, pub
    rospy.init_node('console', anonymous = True)
    rospy.Subscriber("vel", Vector3, velCB)
    rospy.Subscriber("position", Vector3, positionCB)
    rospack = rospkg.RosPack()
    pub = rospy.Publisher('goal', Vector3, queue_size = 1)
    thd = threading.Thread(target = windowShow)
    thd.setDaemon(True)
    thd.start()

def loop():
    global lock, vector3, goal, pub
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        vector3.x = goal[0]
        vector3.y = goal[1]
        vector3.z = goal[2]
        if (lock == False):
            pub.publish(vector3)
        rate.sleep()

def velCB(data):
    global speed
    speed[0] = data.x
    speed[1] = data.y
    speed[2] = data.z

def positionCB(data):
    global robot
    robot[0] = data.x
    robot[1] = data.y
    robot[2] = data.z
    if lock:
        goal[0] = robot[0]
        goal[1] = robot[1]
        goal[2] = robot[2]

def windowShow():
    app = QApplication(sys.argv)
    mainWnd = MainWindow()
    mainWnd.show()
    sys.exit(app.exec_())

class MainWindow(QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle('UESTC Robocon 2018')
        self.resize(MAP_SIZE, MAP_SIZE)
        self.setMouseTracking(True)
        self.timer = QTimer()
        self.timer.timeout.connect(self.repaint)
        self.timer.start(10)
    
    def paintEvent(self, event):
        global ratioX, ratioY, cursor, goal, robot
        ratioX = 14 / self.width()
        ratioY = 14 / self.height()
        self.drawMap()
        self.drawRobot(cursor, Qt.red)
        self.drawRobot(goal, Qt.green)
        self.drawRobot(robot, Qt.blue)
        self.drawPath()
        self.drawText()
    
    def mouseMoveEvent(self, event):
        global stateMachine, ratioX, ratioY, cursor
        if stateMachine == 0:
            cursor[0] = ratioX * event.pos().x()
            cursor[1] = ratioY * (self.height() - event.pos().y())
        elif stateMachine == 1:
            if ratioX * event.pos().x() < cursor[0]:
                cursor[2] = math.atan((ratioY * (self.height() - event.pos().y()) - cursor[1]) / (ratioX * event.pos().x() - cursor[0])) + math.pi / 2
            elif ratioX * event.pos().x() > cursor[0]:
                cursor[2] = math.atan((ratioY * (self.height() - event.pos().y()) - cursor[1]) / (ratioX * event.pos().x() - cursor[0])) - math.pi / 2
            else:
                if ratioY * (self.height() - event.pos().y()) > cursor[1]:
                    cursor[2] = 0
                elif ratioY * (self.height() - event.pos().y()) < cursor[1]:
                    cursor[2] = math.pi
        self.update()
    
    def mousePressEvent(self, event):
        global lock, stateMachine, ratioX, ratioY, cursor, goal
        if stateMachine == 0:
            if event.button() == Qt.LeftButton:
                cursor[0] = ratioX * event.pos().x()
                cursor[1] = ratioY * (self.height() - event.pos().y())
                stateMachine = 1
        elif stateMachine == 1:
            if event.button() == Qt.LeftButton:
                goal[0] = cursor[0]
                goal[1] = cursor[1]
                goal[2] = cursor[2]
                lock = False
            stateMachine = 0
        self.update()
    
    def closeEvent(self, event):
        rospy.signal_shutdown('GUI closed')
    
    def drawMap(self):
        global rospack
        mapPainter = QPainter(self)
        pixmap = QPixmap(rospack.get_path('rc') + "/map.png")
        mapPainter.drawPixmap(self.rect(), pixmap)
    
    def drawRobot(self, point, color):
        global ratioX, ratioY
        robotPainter = QPainter(self)
        robotPainter.translate(point[0] / ratioX, self.height() - point[1] / ratioY)
        robotPainter.rotate(- point[2] * 180 / math.pi)
        arrowPen = QPen(color, 3)
        robotPainter.setPen(arrowPen)
        arrowPts = [QPoint(- ARROW_SIZE / 2, ARROW_SIZE * math.sqrt(3) / 2), QPoint(0, 0), QPoint(ARROW_SIZE / 2, ARROW_SIZE * math.sqrt(3) / 2)]
        robotPainter.drawPolyline(QPolygon(arrowPts))
    
    def drawPath(self):
        global ratioX, ratioY, robot, goal, cursor
        pathPainter = QPainter(self)
        pathPen = QPen(Qt.yellow, 2)
        pathPainter.setPen(pathPen)
        pathPainter.drawLine(robot[0] / ratioX, self.height() - robot[1] / ratioY, goal[0] / ratioX, self.height() - goal[1] / ratioY)
        pathPen = QPen(Qt.yellow, 1)
        pathPainter.setPen(pathPen)
        pathPainter.drawLine(robot[0] / ratioX, self.height() - robot[1] / ratioY, cursor[0] / ratioX, self.height() - cursor[1] / ratioY)
    
    def drawText(self):
        global robot, goal, cursor, speed
        textPainter = QPainter(self)
        textPainter.setPen(Qt.white)
        textPainter.setFont(QFont('等线', 10))
        textPainter.drawText(QPoint(100, 50), "实际坐标: %.3f, %.3f" % (robot[0], robot[1]))
        textPainter.drawText(QPoint(100, 70), "实际角度: %.2f" % (robot[2] * 180 / math.pi))
        textPainter.drawText(QPoint(100, 100), "目的坐标: %.3f, %.3f" % (goal[0], goal[1]))
        textPainter.drawText(QPoint(100, 120), "目的角度: %.2f" % (goal[2] * 180 / math.pi))
        textPainter.drawText(QPoint(100, 150), "指针坐标: %.3f, %.3f" % (cursor[0], cursor[1]))
        textPainter.drawText(QPoint(100, 170), "指针角度: %.2f" % (cursor[2] * 180 / math.pi))
        textPainter.drawText(QPoint(100, 220), "X轴速度: %.3fm/s" % speed[0])
        textPainter.drawText(QPoint(100, 240), "Y轴速度: %.3fm/s" % speed[1])
        textPainter.drawText(QPoint(100, 260), "Z轴速度: %.2fdeg/s" % (speed[2] * 180 / math.pi))
        textPainter.drawText(QPoint(100, 280), "X-Y合速度: %.3fm/s" % math.sqrt(speed[0] ** 2 + speed[1] ** 2))


if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
    loop()
