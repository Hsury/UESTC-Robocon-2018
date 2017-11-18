#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import os
import sys
import math
import bezier
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

PARENT_PATH = os.path.dirname(os.getcwd())

MAP_SIZE = 800
ARROW_SIZE = 15
MARGIN = [1.5, 0.5]
PTS_NUM = 100
DEGREE = 2

keyPts = np.zeros((1, 3))
bezierPts = np.zeros((PTS_NUM, 3))
keyIdx = 0
bezierIdx = -1
stateMachine = 0

def readPts():
    global PARENT_PATH, DEGREE, keyIdx, keyPts
    try:
        fobj = open(PARENT_PATH + '/rc_pts.txt', 'r')
        for eachline in fobj:
            buffer = eachline.split(' ')
            if buffer[0] == 'DEGREE':
                DEGREE = int(buffer[1])
            else:
                keyPts[keyIdx][0] = buffer[0]
                keyPts[keyIdx][1] = buffer[1]
                keyPts[keyIdx][2] = buffer[2]
                keyPts = np.concatenate((keyPts, np.array([keyPts[keyIdx]])))
                keyIdx += 1
        fobj.close()
    except:
        pass
    calcBezier()

def writePts():
    global PARENT_PATH, DEGREE, keyIdx, keyPts, bezierPts
    calcBezier()
    fobj = open(PARENT_PATH + '/rc_bezier.txt', 'w')
    for i in range(PTS_NUM * int((keyIdx - 1) / DEGREE)):
        fobj.writelines('%f %f %f\n' % (bezierPts[i][0], bezierPts[i][1], bezierPts[i][2]))
    fobj.close()
    fobj = open(PARENT_PATH + '/rc_pts.txt', 'w')
    fobj.writelines('DEGREE %d\n' % DEGREE)
    for i in range(keyIdx):
        fobj.writelines('%f %f %f\n' % (keyPts[i][0], keyPts[i][1], keyPts[i][2]))
    fobj.close()

def windowShow():
    app = QApplication(sys.argv)
    mainWnd = MainWindow()
    mainWnd.show()
    sys.exit(app.exec_())

def calcBezier():
    global PTS_NUM, DEGREE, keyIdx, keyPts, bezierPts
    for i in range(int(keyIdx / DEGREE)):
        if keyPts[i * DEGREE][2] < 0:
            keyPts[i * DEGREE][2] += 2 * math.pi
        for j in range(1, DEGREE + 1):
            delta = keyPts[i * DEGREE + j][2] - keyPts[i * DEGREE + j - 1][2]
            if math.fabs(delta) > math.fabs(delta + 2 * math.pi):
                keyPts[i * DEGREE + j][2] += 2 * math.pi
        curve = bezier.Curve(keyPts[i * DEGREE : (i + 1) * DEGREE + 1, :], degree = DEGREE)
        s_vals = np.linspace(0.0, 1.0, PTS_NUM)
        if i == 0:
            bezierPts = curve.evaluate_multi(s_vals)
        else:
            bezierPts = np.concatenate((bezierPts, curve.evaluate_multi(s_vals)))
        for j in range(DEGREE + 1):
            if keyPts[i * DEGREE + j][2] >= math.pi:
                keyPts[i * DEGREE + j][2] -= 2 * math.pi
        for j in range(PTS_NUM):
            if bezierPts[i * PTS_NUM + j][2] >= math.pi:
                bezierPts[i * PTS_NUM + j][2] -= 2 * math.pi

class MainWindow(QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle('UESTC Robocon 2018 - 路径编辑器')
        self.resize(MAP_SIZE, MAP_SIZE)
        self.setMouseTracking(True)
        self.timer = QTimer()
        self.timer.timeout.connect(self.ptsGo)
        self.timer.start(10)
    
    def ptsGo(self):
        global bezierIdx
        bezierIdx += 1
        self.update()

    def paintEvent(self, event):
        global DEGREE, ratioX, ratioY, keyIdx
        ratioX = 14 / self.width()
        ratioY = 14 / self.height()
        if (keyIdx % DEGREE == 0):
            calcBezier()
        self.drawMap()
        self.drawLinePath()
        self.drawBezierPath()
        self.drawPoint()
        self.drawSim()
        self.drawText()
    
    def mouseMoveEvent(self, event):
        global DEGREE, stateMachine, ratioX, ratioY, keyIdx, keyPts
        if stateMachine == 0:
            keyPts[keyIdx][0] = ratioX * event.pos().x()
            keyPts[keyIdx][1] = ratioY * (self.height() - event.pos().y())
        elif stateMachine == 1:
            if ratioX * event.pos().x() < keyPts[keyIdx][0]:
                keyPts[keyIdx][2] = math.atan((ratioY * (self.height() - event.pos().y()) - keyPts[keyIdx][1]) / (ratioX * event.pos().x() - keyPts[keyIdx][0])) + math.pi / 2
            elif ratioX * event.pos().x() > keyPts[keyIdx][0]:
                keyPts[keyIdx][2] = math.atan((ratioY * (self.height() - event.pos().y()) - keyPts[keyIdx][1]) / (ratioX * event.pos().x() - keyPts[keyIdx][0])) - math.pi / 2
            else:
                if ratioY * (self.height() - event.pos().y()) > keyPts[keyIdx][1]:
                    keyPts[keyIdx][2] = 0
                elif ratioY * (self.height() - event.pos().y()) < keyPts[keyIdx][1]:
                    keyPts[keyIdx][2] = - math.pi
        self.update()
    
    def mousePressEvent(self, event):
        global DEGREE, stateMachine, ratioX, ratioY, keyIdx, keyPts
        if stateMachine == 0:
            if event.button() == Qt.LeftButton:
                stateMachine = 1
            elif event.button() == Qt.RightButton:
                if keyIdx >= 1:
                    keyPts = np.delete(keyPts, -1, 0)
                    keyIdx -= 1
                    if ratioX * event.pos().x() < keyPts[keyIdx][0]:
                        keyPts[keyIdx][2] = math.atan((ratioY * (self.height() - event.pos().y()) - keyPts[keyIdx][1]) / (ratioX * event.pos().x() - keyPts[keyIdx][0])) + math.pi / 2
                    elif ratioX * event.pos().x() > keyPts[keyIdx][0]:
                        keyPts[keyIdx][2] = math.atan((ratioY * (self.height() - event.pos().y()) - keyPts[keyIdx][1]) / (ratioX * event.pos().x() - keyPts[keyIdx][0])) - math.pi / 2
                    else:
                        if ratioY * (self.height() - event.pos().y()) > keyPts[keyIdx][1]:
                            keyPts[keyIdx][2] = 0
                        elif ratioY * (self.height() - event.pos().y()) < keyPts[keyIdx][1]:
                            keyPts[keyIdx][2] = - math.pi
                    stateMachine = 1
        elif stateMachine == 1:
            if event.button() == Qt.LeftButton:
                keyPts = np.concatenate((keyPts, np.array([keyPts[keyIdx]])))
                keyIdx += 1
                stateMachine = 0
            elif event.button() == Qt.RightButton:
                if keyIdx >= 1:
                    keyPts[keyIdx][2] = keyPts[keyIdx - 1][2]
                else:
                    keyPts[keyIdx][2] = 0
                stateMachine = 0
            keyPts[keyIdx][0] = ratioX * event.pos().x()
            keyPts[keyIdx][1] = ratioY * (self.height() - event.pos().y())
        self.update()
    
    def closeEvent(self, event):
        writePts()
    
    def drawMap(self):
        global PARENT_PATH
        mapPainter = QPainter(self)
        pixmap = QPixmap(PARENT_PATH + "/map.png")
        mapPainter.drawPixmap(self.rect(), pixmap)
    
    def drawPoint(self):
        global stateMachine, ratioX, ratioY, keyIdx, keyPts
        ptPainter = QPainter(self)
        arrowPts = [QPoint(- ARROW_SIZE / 2, ARROW_SIZE * math.sqrt(3) / 2), QPoint(0, 0), QPoint(ARROW_SIZE / 2, ARROW_SIZE * math.sqrt(3) / 2)]
        for i in range(keyIdx + 1):
            ptPainter.save()
            ptPainter.translate(keyPts[i][0] / ratioX, self.height() - keyPts[i][1] / ratioY)
            ptPainter.rotate(- keyPts[i][2] * 180 / math.pi)
            if (i == keyIdx):
                if (stateMachine == 0):
                    arrowPen = QPen(Qt.red, 2)
                else:
                    arrowPen = QPen(Qt.magenta, 2)
            else:
                if (i % DEGREE == 0):
                    arrowPen = QPen(Qt.green, 2)
                else:
                    arrowPen = QPen(Qt.blue, 2)
            ptPainter.setPen(arrowPen)
            ptPainter.drawPolyline(QPolygon(arrowPts))
            if (i % DEGREE == 0):
                ptPainter.drawEllipse(-3, -3, 6, 6)
            ptPainter.restore()
    
    def drawSim(self):
        global DEGREE, ratioX, ratioY, keyIdx, bezierIdx, bezierPts
        simPainter = QPainter(self)
        arrowPen = QPen(Qt.cyan, 2)
        simPainter.setPen(arrowPen)
        arrowPts = [QPoint(- ARROW_SIZE / 2, ARROW_SIZE * math.sqrt(3) / 2), QPoint(0, 0), QPoint(ARROW_SIZE / 2, ARROW_SIZE * math.sqrt(3) / 2)]
        if bezierIdx < PTS_NUM * int(keyIdx / DEGREE) - 1:
            simPainter.translate(bezierPts[bezierIdx][0] / ratioX, self.height() - bezierPts[bezierIdx][1] / ratioY)
            simPainter.rotate(- bezierPts[bezierIdx][2] * 180 / math.pi)
            simPainter.drawPolyline(QPolygon(arrowPts))
            simPainter.drawEllipse(-2, -2, 4, 4)
        else:
            bezierIdx = 0
    
    def drawLinePath(self):
        global stateMachine, ratioX, ratioY, keyIdx, keyPts
        if (keyIdx >= 1):
            linePathPainter = QPainter(self)
            pathPen = QPen(Qt.yellow, 1)
            linePathPainter.setPen(pathPen)
            for i in range(keyIdx - 1):
                linePathPainter.drawLine(keyPts[i][0] / ratioX, self.height() - keyPts[i][1] / ratioY, keyPts[i + 1][0] / ratioX, self.height() - keyPts[i + 1][1] / ratioY)
            if stateMachine == 0:
                pathPen = QPen(Qt.yellow, 0.5)
                linePathPainter.setPen(pathPen)
            linePathPainter.drawLine(keyPts[keyIdx - 1][0] / ratioX, self.height() - keyPts[keyIdx - 1][1] / ratioY, keyPts[keyIdx][0] / ratioX, self.height() - keyPts[keyIdx][1] / ratioY)

    def drawBezierPath(self):
        global PTS_NUM, DEGREE, ratioX, ratioY, keyIdx, bezierPts
        bezierPathPainter = QPainter(self)
        pathPen = QPen(Qt.green, 2)
        bezierPathPainter.setPen(pathPen)
        for i in range(PTS_NUM * int(keyIdx / DEGREE) - 1):
            bezierPathPainter.drawLine(bezierPts[i][0] / ratioX, self.height() - bezierPts[i][1] / ratioY, bezierPts[i + 1][0] / ratioX, self.height() - bezierPts[i + 1][1] / ratioY)
    
    def drawText(self):
        global MARGIN, PTS_NUM, DEGREE, keyIdx, bezierIdx, keyPts, bezierPts
        textPainter = QPainter(self)
        textPainter.setPen(Qt.white)
        textPainter.setFont(QFont('等线', 10))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  20), "模式: %d阶贝塞尔曲线" % DEGREE)
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  40), "段数: %d" % int((keyIdx - 1) / DEGREE))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  60), "点数: %d => %d" % (keyIdx, PTS_NUM * int(keyIdx / DEGREE)))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  100), "仿真点")
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  120), "X: %.3f m" % bezierPts[bezierIdx][0])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  140), "Y: %.3f m" % bezierPts[bezierIdx][1])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  160), "Z: %.2f deg" % (bezierPts[bezierIdx][2] / math.pi * 180))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  200), "指针: %s" % ('端点' if keyIdx % DEGREE == 0 else '控制点'))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  220), "X: %.3f m" % keyPts[keyIdx][0])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  240), "Y: %.3f m" % keyPts[keyIdx][1])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  260), "Z: %.2f deg" % (keyPts[keyIdx][2] / math.pi * 180))

if __name__ == '__main__':
    readPts()
    windowShow()
