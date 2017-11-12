#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import os
import sys
import rospy
import math
import threading
import numpy as np
from geometry_msgs.msg import Vector3
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

PARENT_PATH = os.path.dirname(os.getcwd())

MAP_SIZE = 800
ARROW_SIZE = 15
MARGIN = [1.5, 0.5]

schePts = np.zeros((1, 3))
realPts = np.zeros((1, 3))
cursor = [0] * 3
speed = [0] * 3
goal = [0] * 3
lock = True
stateMachine = 0
vector3 = Vector3()

def setup():
    global pub
    rospy.init_node('console', anonymous = True)
    rospy.Subscriber("vel", Vector3, velCB)
    rospy.Subscriber("position", Vector3, positionCB)
    pub = rospy.Publisher('goal', Vector3, queue_size = 1)
    readPts()
    thd = threading.Thread(target = windowShow)
    thd.setDaemon(True)
    thd.start()

def loop():
    global lock, vector3, goal, pub
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if (lock == False):
            vector3.x = goal[0]
            vector3.y = goal[1]
            vector3.z = goal[2]
            pub.publish(vector3)
            lock = True
        rate.sleep()

def readPts():
    global PARENT_PATH, schePts
    try:
        fobj = open(PARENT_PATH + '/rc_bezier.txt', 'r')
        idx = 0
        for eachline in fobj:
            buffer = eachline.encode('utf-8').split(' ')
            schePts[idx][0] = buffer[0]
            schePts[idx][1] = buffer[1]
            schePts[idx][2] = buffer[2]
            schePts = np.concatenate((schePts, np.array([schePts[idx]])))
            idx += 1
        fobj.close()
    except:
        pass

def velCB(data):
    global speed
    speed[0] = data.x
    speed[1] = data.y
    speed[2] = data.z

def positionCB(data):
    global realPts
    if math.fabs(realPts[-1][0] - data.x) > 0.001 or math.fabs(realPts[-1][1] - data.y) > 0.001 or math.fabs(realPts[-1][2] - data.z) > 0.01:
       realPts[-1][0] = data.x
       realPts[-1][1] = data.y
       realPts[-1][2] = data.z
       realPts = np.concatenate((realPts, np.array([realPts[-1]])))

def windowShow():
    app = QApplication(sys.argv)
    mainWnd = MainWindow()
    mainWnd.show()
    sys.exit(app.exec_())

class MainWindow(QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle('UESTC Robocon 2018 - 控制台')
        self.resize(MAP_SIZE, MAP_SIZE)
        self.setMouseTracking(True)
        self.timer = QTimer()
        self.timer.timeout.connect(self.repaint)
        self.timer.start(10)

    def paintEvent(self, event):
        global ratioX, ratioY
        ratioX = 14 / self.width()
        ratioY = 14 / self.height()
        self.drawMap()
        self.drawPath()
        self.drawPoint()
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
                    cursor[2] = - math.pi
        self.update()
    
    def mousePressEvent(self, event):
        global lock, stateMachine, ratioX, ratioY, goal, cursor
        if stateMachine == 0:
            if event.button() == Qt.LeftButton:
                stateMachine = 1   
        elif stateMachine == 1:
            if event.button() == Qt.LeftButton:
                goal[0] = cursor[0]
                goal[1] = cursor[1]
                goal[2] = cursor[2]
                lock = False
                stateMachine = 0
            elif event.button() == Qt.RightButton:
                cursor[2] = 0
                stateMachine = 0
            cursor[0] = ratioX * event.pos().x()
            cursor[1] = ratioY * (self.height() - event.pos().y())
        self.update()
    
    def closeEvent(self, event):
        rospy.signal_shutdown('GUI closed')
    
    def drawMap(self):
        global PARENT_PATH
        mapPainter = QPainter(self)
        pixmap = QPixmap(PARENT_PATH + "/map.png")
        mapPainter.drawPixmap(self.rect(), pixmap)
    
    def drawPoint(self):
        global stateMachine, ratioX, ratioY, realPts, cursor, goal
        ptPainter = QPainter(self)
        arrowPts = [QPoint(- ARROW_SIZE / 2, ARROW_SIZE * math.sqrt(3) / 2), QPoint(0, 0), QPoint(ARROW_SIZE / 2, ARROW_SIZE * math.sqrt(3) / 2)]
        ptPainter.save()
        ptPainter.translate(goal[0] / ratioX, self.height() - goal[1] / ratioY)
        ptPainter.rotate(- goal[2] * 180 / math.pi)
        arrowPen = QPen(Qt.blue, 2)
        ptPainter.setPen(arrowPen)
        ptPainter.drawPolyline(QPolygon(arrowPts))
        ptPainter.restore()
        ptPainter.save()
        ptPainter.translate(realPts[-1][0] / ratioX, self.height() - realPts[-1][1] / ratioY)
        ptPainter.rotate(- realPts[-1][2] * 180 / math.pi)
        arrowPen = QPen(Qt.green, 2)
        ptPainter.setPen(arrowPen)
        ptPainter.drawPolyline(QPolygon(arrowPts))
        ptPainter.restore()
        ptPainter.save()
        ptPainter.translate(cursor[0] / ratioX, self.height() - cursor[1] / ratioY)
        ptPainter.rotate(- cursor[2] * 180 / math.pi)
        if (stateMachine == 0):
            arrowPen = QPen(Qt.red, 2)
        else:
            arrowPen = QPen(Qt.magenta, 2)
        ptPainter.setPen(arrowPen)
        ptPainter.drawPolyline(QPolygon(arrowPts))
        ptPainter.restore()

    def drawPath(self):
        global ratioX, ratioY, schePts, realPts, goal, cursor
        pathPainter = QPainter(self)
        pathPen = QPen(Qt.green, 1)
        pathPainter.setPen(pathPen)
        for i in range(schePts.shape[0] - 1):
            pathPainter.drawLine(schePts[i][0] / ratioX, self.height() - schePts[i][1] / ratioY, schePts[i + 1][0] / ratioX, self.height() - schePts[i + 1][1] / ratioY)
        pathPen = QPen(Qt.red, 1)
        pathPainter.setPen(pathPen)
        for i in range(realPts.shape[0] - 1):
            pathPainter.drawLine(realPts[i][0] / ratioX, self.height() - realPts[i][1] / ratioY, realPts[i + 1][0] / ratioX, self.height() - realPts[i + 1][1] / ratioY)
        pathPen = QPen(Qt.yellow, 1)
        pathPainter.setPen(pathPen)
        pathPainter.drawLine(realPts[-1][0] / ratioX, self.height() - realPts[-1][1] / ratioY, goal[0] / ratioX, self.height() - goal[1] / ratioY)
        pathPen = QPen(Qt.yellow, 0.5)
        pathPainter.setPen(pathPen)
        pathPainter.drawLine(realPts[-1][0] / ratioX, self.height() - realPts[-1][1] / ratioY, cursor[0] / ratioX, self.height() - cursor[1] / ratioY)

    def drawText(self):
        global MARGIN, realPts, cursor, speed, goal
        textPainter = QPainter(self)
        textPainter.setPen(Qt.white)
        textPainter.setFont(QFont('等线', 10))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  20), "实际")
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  40), "X: %.3f m" % realPts[-1][0])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  60), "Y: %.3f m" % realPts[-1][1])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  80), "Z: %.2f deg" % (realPts[-1][2] / math.pi * 180))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  120), "目的")
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  140), "X: %.3f m" % goal[0])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  160), "Y: %.3f m" % goal[1])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  180), "Z: %.2f deg" % (goal[2] / math.pi * 180))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  220), "指针")
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  240), "X: %.3f m" % cursor[0])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  260), "Y: %.3f m" % cursor[1])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  280), "Z: %.2f deg" % (cursor[2] / math.pi * 180))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  320), "速度")
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  340), "X-Y: %.3f m/s" % math.sqrt(speed[0] ** 2 + speed[1] ** 2))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  360), "Z: %.2f deg/s" % (speed[2] / math.pi * 180))

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
    loop()
