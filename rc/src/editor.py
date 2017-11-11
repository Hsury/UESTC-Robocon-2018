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

origPts = np.zeros((1, 3))
bezierPts = np.zeros((PTS_NUM, 3))
idx = 0
simidx = -1
stateMachine = 0

def readPts():
    global PARENT_PATH, DEGREE, idx, origPts
    try:
        fobj = open(PARENT_PATH + '/rc_pts.txt', 'r')
        for eachline in fobj:
            buffer = eachline.split(' ')
            if buffer[0] == 'DEGREE':
                DEGREE = int(buffer[1])
            else:
                origPts[idx][0] = buffer[0]
                origPts[idx][1] = buffer[1]
                origPts[idx][2] = buffer[2]
                origPts = np.concatenate((origPts, np.array([origPts[idx]])))
                idx += 1
        fobj.close()
    except:
        pass
    calcBezier()

def writePts():
    global PARENT_PATH, DEGREE, idx, origPts, bezierPts
    calcBezier()
    fobj = open(PARENT_PATH + '/rc_bezier.txt', 'w')
    for i in range(PTS_NUM * int((idx - 1) / DEGREE)):
        fobj.writelines('%f %f %f\n' % (bezierPts[i][0], bezierPts[i][1], bezierPts[i][2]))
    fobj.close()
    fobj = open(PARENT_PATH + '/rc_pts.txt', 'w')
    fobj.writelines('DEGREE %d\n' % DEGREE)
    for i in range(idx):
        fobj.writelines('%f %f %f\n' % (origPts[i][0], origPts[i][1], origPts[i][2]))
    fobj.close()

def windowShow():
    app = QApplication(sys.argv)
    mainWnd = MainWindow()
    mainWnd.show()
    sys.exit(app.exec_())

def calcBezier():
    global PTS_NUM, DEGREE, idx, origPts, bezierPts
    for i in range(int(idx / DEGREE)):
        if origPts[i * DEGREE][2] < 0:
            origPts[i * DEGREE][2] += 2 * math.pi
        for j in range(1, DEGREE + 1):
            delta = origPts[i * DEGREE + j][2] - origPts[i * DEGREE + j - 1][2]
            if math.fabs(delta) > math.fabs(delta + 2 * math.pi):
                origPts[i * DEGREE + j][2] += 2 * math.pi
        curve = bezier.Curve(origPts[i * DEGREE : (i + 1) * DEGREE + 1, :], degree = DEGREE)
        s_vals = np.linspace(0.0, 1.0, PTS_NUM)
        if i == 0:
            bezierPts = curve.evaluate_multi(s_vals)
        else:
            bezierPts = np.concatenate((bezierPts, curve.evaluate_multi(s_vals)))
        for j in range(DEGREE + 1):
            if origPts[i * DEGREE + j][2] >= math.pi:
                origPts[i * DEGREE + j][2] -= 2 * math.pi
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
        global simidx
        simidx += 1
        self.update()

    def paintEvent(self, event):
        global DEGREE, ratioX, ratioY, idx
        ratioX = 14 / self.width()
        ratioY = 14 / self.height()
        if (idx % DEGREE == 0):
            calcBezier()
        self.drawMap()
        self.drawLinePath()
        self.drawBezierPath()
        self.drawPoint()
        self.drawSim()
        self.drawText()
    
    def mouseMoveEvent(self, event):
        global DEGREE, stateMachine, ratioX, ratioY, idx, origPts
        if stateMachine == 0:
            origPts[idx][0] = ratioX * event.pos().x()
            origPts[idx][1] = ratioY * (self.height() - event.pos().y())
        elif stateMachine == 1:
            if ratioX * event.pos().x() < origPts[idx][0]:
                origPts[idx][2] = math.atan((ratioY * (self.height() - event.pos().y()) - origPts[idx][1]) / (ratioX * event.pos().x() - origPts[idx][0])) + math.pi / 2
            elif ratioX * event.pos().x() > origPts[idx][0]:
                origPts[idx][2] = math.atan((ratioY * (self.height() - event.pos().y()) - origPts[idx][1]) / (ratioX * event.pos().x() - origPts[idx][0])) - math.pi / 2
            else:
                if ratioY * (self.height() - event.pos().y()) > origPts[idx][1]:
                    origPts[idx][2] = 0
                elif ratioY * (self.height() - event.pos().y()) < origPts[idx][1]:
                    origPts[idx][2] = - math.pi
        self.update()
    
    def mousePressEvent(self, event):
        global DEGREE, stateMachine, ratioX, ratioY, idx, origPts
        if stateMachine == 0:
            if event.button() == Qt.LeftButton:
                stateMachine = 1
            elif event.button() == Qt.RightButton:
                if idx >= 1:
                    origPts = np.delete(origPts, -1, 0)
                    idx -= 1
                    if ratioX * event.pos().x() < origPts[idx][0]:
                        origPts[idx][2] = math.atan((ratioY * (self.height() - event.pos().y()) - origPts[idx][1]) / (ratioX * event.pos().x() - origPts[idx][0])) + math.pi / 2
                    elif ratioX * event.pos().x() > origPts[idx][0]:
                        origPts[idx][2] = math.atan((ratioY * (self.height() - event.pos().y()) - origPts[idx][1]) / (ratioX * event.pos().x() - origPts[idx][0])) - math.pi / 2
                    else:
                        if ratioY * (self.height() - event.pos().y()) > origPts[idx][1]:
                            origPts[idx][2] = 0
                        elif ratioY * (self.height() - event.pos().y()) < origPts[idx][1]:
                            origPts[idx][2] = - math.pi
                    stateMachine = 1
        elif stateMachine == 1:
            if event.button() == Qt.LeftButton:
                origPts = np.concatenate((origPts, np.array([origPts[idx]])))
                idx += 1
                stateMachine = 0
            elif event.button() == Qt.RightButton:
                if idx >= 1:
                    origPts[idx][2] = origPts[idx - 1][2]
                else:
                    origPts[idx][2] = 0
                stateMachine = 0
            origPts[idx][0] = ratioX * event.pos().x()
            origPts[idx][1] = ratioY * (self.height() - event.pos().y())
        self.update()
    
    def closeEvent(self, event):
        writePts()
    
    def drawMap(self):
        global PARENT_PATH
        mapPainter = QPainter(self)
        pixmap = QPixmap(PARENT_PATH + "/map.png")
        mapPainter.drawPixmap(self.rect(), pixmap)
    
    def drawPoint(self):
        global stateMachine, ratioX, ratioY, idx, origPts
        ptPainter = QPainter(self)
        arrowPts = [QPoint(- ARROW_SIZE / 2, ARROW_SIZE * math.sqrt(3) / 2), QPoint(0, 0), QPoint(ARROW_SIZE / 2, ARROW_SIZE * math.sqrt(3) / 2)]
        for i in range(idx + 1):
            ptPainter.save()
            ptPainter.translate(origPts[i][0] / ratioX, self.height() - origPts[i][1] / ratioY)
            ptPainter.rotate(- origPts[i][2] * 180 / math.pi)
            if (i == idx):
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
        global DEGREE, ratioX, ratioY, idx, simidx, bezierPts
        simPainter = QPainter(self)
        arrowPen = QPen(Qt.cyan, 2)
        simPainter.setPen(arrowPen)
        arrowPts = [QPoint(- ARROW_SIZE / 2, ARROW_SIZE * math.sqrt(3) / 2), QPoint(0, 0), QPoint(ARROW_SIZE / 2, ARROW_SIZE * math.sqrt(3) / 2)]
        if simidx < PTS_NUM * int(idx / DEGREE) - 1:
            simPainter.translate(bezierPts[simidx][0] / ratioX, self.height() - bezierPts[simidx][1] / ratioY)
            simPainter.rotate(- bezierPts[simidx][2] * 180 / math.pi)
            simPainter.drawPolyline(QPolygon(arrowPts))
            simPainter.drawEllipse(-2, -2, 4, 4)
        else:
            simidx = 0
    
    def drawLinePath(self):
        global stateMachine, ratioX, ratioY, idx, origPts
        if (idx >= 1):
            linePathPainter = QPainter(self)
            pathPen = QPen(Qt.yellow, 1)
            linePathPainter.setPen(pathPen)
            for i in range(idx - 1):
                linePathPainter.drawLine(origPts[i][0] / ratioX, self.height() - origPts[i][1] / ratioY, origPts[i + 1][0] / ratioX, self.height() - origPts[i + 1][1] / ratioY)
            if stateMachine == 0:
                pathPen = QPen(Qt.yellow, 0.5)
                linePathPainter.setPen(pathPen)
            linePathPainter.drawLine(origPts[idx - 1][0] / ratioX, self.height() - origPts[idx - 1][1] / ratioY, origPts[idx][0] / ratioX, self.height() - origPts[idx][1] / ratioY)

    def drawBezierPath(self):
        global PTS_NUM, DEGREE, ratioX, ratioY, idx, bezierPts
        bezierPathPainter = QPainter(self)
        pathPen = QPen(Qt.green, 2)
        bezierPathPainter.setPen(pathPen)
        for i in range(PTS_NUM * int(idx / DEGREE) - 1):
            bezierPathPainter.drawLine(bezierPts[i][0] / ratioX, self.height() - bezierPts[i][1] / ratioY, bezierPts[i + 1][0] / ratioX, self.height() - bezierPts[i + 1][1] / ratioY)
    
    def drawText(self):
        global MARGIN, PTS_NUM, DEGREE, idx, subidx, origPts, bezierPts
        textPainter = QPainter(self)
        textPainter.setPen(Qt.white)
        textPainter.setFont(QFont('等线', 10))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  20), "模式: %d阶贝塞尔曲线" % DEGREE)
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  40), "段数: %d" % int((idx - 1) / DEGREE))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  60), "点数: %d => %d" % (idx, PTS_NUM * int(idx / DEGREE)))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  100), "仿真点")
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  120), "X: %.3f m" % bezierPts[simidx][0])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  140), "Y: %.3f m" % bezierPts[simidx][1])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  160), "Z: %.2f deg" % (bezierPts[simidx][2] / math.pi * 180))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  200), "指针: %s" % ('端点' if idx % DEGREE == 0 else '控制点'))
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  220), "X: %.3f m" % origPts[idx][0])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  240), "Y: %.3f m" % origPts[idx][1])
        textPainter.drawText(QPoint(MARGIN[0] / ratioX, MARGIN[1] / ratioY +  260), "Z: %.2f deg" % (origPts[idx][2] / math.pi * 180))

if __name__ == '__main__':
    readPts()
    windowShow()
