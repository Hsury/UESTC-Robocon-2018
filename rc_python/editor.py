from math import sin, cos, atan, pi, sqrt, fabs
import multiprocessing as mp
import sys
import os
import bezier
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class Editor():
    '''
    UESTC 2018 Robocon Team
    Editor
    '''
    def __init__(self, degree=3, sampling=500):
        try:
            mp.set_start_method('spawn')
        except:
            pass
        p = mp.Process(target=EditorWindow, args=(degree, sampling))
        p.start()

class EditorWindow(QWidget):
    MAP_SIZE = 800
    ARROW_SIZE = 15
    MARGIN = [1.5, 0.5]

    def __init__(self, degree=3, sampling=500):
        app = QApplication(sys.argv)
        super(EditorWindow, self).__init__()
        self._degree = degree
        self._sampling = sampling
        self._dataDir =  os.getcwd() + os.sep + 'data'  #To Get Parent Directory Use os.path.dirname(os.getcwd())
        self._endList = np.zeros((1, 3))
        self._bezierList = np.zeros((self._sampling, 3))
        self._endIdx = 0
        self._bezierIdx = -1
        self._stateMachine = 0
        self.__loadPath()
        self.setWindowTitle('UESTC Robocon 2018 - 路径编辑器')
        self.resize(EditorWindow.MAP_SIZE, EditorWindow.MAP_SIZE)
        self.setMouseTracking(True)
        self.timer = QTimer()
        self.timer.timeout.connect(self.__timer)
        self.timer.start(10)
        self.show()
        sys.exit(app.exec_())
    
    def paintEvent(self, event):
        self._ratioX = 14 / self.width()
        self._ratioY = 14 / self.height()
        if self._endIdx % self._degree == 0:
            self.__calcBezier()
        self.__drawMap()
        self.__drawLinePath()
        self.__drawBezierPath()
        self.__drawPoint()
        self.__drawSim()
        self.__drawText()
    
    def mouseMoveEvent(self, event):
        if self._stateMachine == 0:
            self._endList[self._endIdx][0] = self._ratioX * event.pos().x()
            self._endList[self._endIdx][1] = self._ratioY * (self.height() - event.pos().y())
        elif self._stateMachine == 1:
            if self._ratioX * event.pos().x() < self._endList[self._endIdx][0]:
                self._endList[self._endIdx][2] = atan((self._ratioY * (self.height() - event.pos().y()) - self._endList[self._endIdx][1]) / (self._ratioX * event.pos().x() - self._endList[self._endIdx][0])) + pi / 2
            elif self._ratioX * event.pos().x() > self._endList[self._endIdx][0]:
                self._endList[self._endIdx][2] = atan((self._ratioY * (self.height() - event.pos().y()) - self._endList[self._endIdx][1]) / (self._ratioX * event.pos().x() - self._endList[self._endIdx][0])) - pi / 2
            else:
                if self._ratioY * (self.height() - event.pos().y()) > self._endList[self._endIdx][1]:
                    self._endList[self._endIdx][2] = 0
                elif self._ratioY * (self.height() - event.pos().y()) < self._endList[self._endIdx][1]:
                    self._endList[self._endIdx][2] = - pi
    
    def mousePressEvent(self, event):
        if self._stateMachine == 0:
            if event.button() == Qt.LeftButton:
                self._stateMachine = 1
            elif event.button() == Qt.RightButton:
                if self._endIdx >= 1:
                    self._endList = np.delete(self._endList, -1, 0)
                    self._endIdx -= 1
                    if self._ratioX * event.pos().x() < self._endList[self._endIdx][0]:
                        self._endList[self._endIdx][2] = atan((self._ratioY * (self.height() - event.pos().y()) - self._endList[self._endIdx][1]) / (self._ratioX * event.pos().x() - self._endList[self._endIdx][0])) + pi / 2
                    elif self._ratioX * event.pos().x() > self._endList[self._endIdx][0]:
                        self._endList[self._endIdx][2] = atan((self._ratioY * (self.height() - event.pos().y()) - self._endList[self._endIdx][1]) / (self._ratioX * event.pos().x() - self._endList[self._endIdx][0])) - pi / 2
                    else:
                        if self._ratioY * (self.height() - event.pos().y()) > self._endList[self._endIdx][1]:
                            self._endList[self._endIdx][2] = 0
                        elif self._ratioY * (self.height() - event.pos().y()) < self._endList[self._endIdx][1]:
                            self._endList[self._endIdx][2] = - pi
                    self._stateMachine = 1
        elif self._stateMachine == 1:
            if event.button() == Qt.LeftButton:
                self._endList = np.concatenate((self._endList, np.array([self._endList[self._endIdx]])))
                self._endIdx += 1
                self._stateMachine = 0
            elif event.button() == Qt.RightButton:
                if self._endIdx >= 1:
                    self._endList[self._endIdx][2] = self._endList[self._endIdx - 1][2]
                else:
                    self._endList[self._endIdx][2] = 0
                self._stateMachine = 0
            self._endList[self._endIdx][0] = self._ratioX * event.pos().x()
            self._endList[self._endIdx][1] = self._ratioY * (self.height() - event.pos().y())
    
    def closeEvent(self, event):
        self.__savePath()
    
    def __loadPath(self):
        try:
            with open(self._dataDir + os.sep + 'rc_points.txt', 'r') as fobj:
                for eachline in fobj:
                    buffer = [float(data) for data in eachline.split(' ')]
                    self._endList[self._endIdx] = buffer[:]
                    self._endList = np.concatenate((self._endList, np.array([self._endList[self._endIdx]])))
                    self._endIdx += 1
        except:
            pass
        self.__calcBezier()
    
    def __savePath(self):
        self.__calcBezier()
        try:
            with open(self._dataDir + os.sep + 'rc_points.txt', 'w') as fobj:
                for idx in range(self._endIdx):
                    fobj.writelines('{} {} {}\n'.format(self._endList[idx][0], self._endList[idx][1], self._endList[idx][2]))
            with open(self._dataDir + os.sep + 'rc_bezier.txt', 'w') as fobj:
                for idx in range(self._sampling * int((self._endIdx - 1) / self._degree)):
                    fobj.writelines('{} {} {}\n'.format(self._bezierList[idx][0], self._bezierList[idx][1], self._bezierList[idx][2]))
        except:
            pass
    
    def __timer(self):
        self._bezierIdx += 1
        self.update()
    
    def __calcBezier(self):
        for i in range(int(self._endIdx / self._degree)):
            if self._endList[i * self._degree][2] < 0:
                self._endList[i * self._degree][2] += 2 * pi
            for j in range(1, self._degree + 1):
                delta = self._endList[i * self._degree + j][2] - self._endList[i * self._degree + j - 1][2]
                if fabs(delta) > fabs(delta + 2 * pi):
                    self._endList[i * self._degree + j][2] += 2 * pi
            curve = bezier.Curve(self._endList[i * self._degree : (i + 1) * self._degree + 1, :], degree=self._degree)
            s_vals = np.linspace(0.0, 1.0, self._sampling)
            if i == 0:
                self._bezierList = curve.evaluate_multi(s_vals)
            else:
                self._bezierList = np.concatenate((self._bezierList, curve.evaluate_multi(s_vals)))
            for j in range(self._degree + 1):
                if self._endList[i * self._degree + j][2] >= pi:
                    self._endList[i * self._degree + j][2] -= 2 * pi
            for j in range(self._sampling):
                if self._bezierList[i * self._sampling + j][2] >= pi:
                    self._bezierList[i * self._sampling + j][2] -= 2 * pi
    
    def __drawMap(self):
        mapPainter = QPainter(self)
        pixmap = QPixmap(self._dataDir + os.sep + "map.png")
        mapPainter.drawPixmap(self.rect(), pixmap)
    
    def __drawPoint(self):
        ptPainter = QPainter(self)
        arrowPts = [QPoint(- EditorWindow.ARROW_SIZE / 2, EditorWindow.ARROW_SIZE * sqrt(3) / 2), QPoint(0, 0), QPoint(EditorWindow.ARROW_SIZE / 2, EditorWindow.ARROW_SIZE * sqrt(3) / 2)]
        for i in range(self._endIdx + 1):
            ptPainter.save()
            ptPainter.translate(self._endList[i][0] / self._ratioX, self.height() - self._endList[i][1] / self._ratioY)
            ptPainter.rotate(- self._endList[i][2] * 180 / pi)
            if (i == self._endIdx):
                if (self._stateMachine == 0):
                    arrowPen = QPen(Qt.red, 2)
                else:
                    arrowPen = QPen(Qt.magenta, 2)
            else:
                if (i % self._degree == 0):
                    arrowPen = QPen(Qt.green, 2)
                else:
                    arrowPen = QPen(Qt.blue, 2)
            ptPainter.setPen(arrowPen)
            ptPainter.drawPolyline(QPolygon(arrowPts))
            if (i % self._degree == 0):
                ptPainter.drawEllipse(-3, -3, 6, 6)
            ptPainter.restore()
    
    def __drawSim(self):
        simPainter = QPainter(self)
        arrowPen = QPen(Qt.cyan, 2)
        simPainter.setPen(arrowPen)
        arrowPts = [QPoint(- EditorWindow.ARROW_SIZE / 2, EditorWindow.ARROW_SIZE * sqrt(3) / 2), QPoint(0, 0), QPoint(EditorWindow.ARROW_SIZE / 2, EditorWindow.ARROW_SIZE * sqrt(3) / 2)]
        if self._bezierIdx < self._sampling * int(self._endIdx / self._degree) - 1:
            simPainter.translate(self._bezierList[self._bezierIdx][0] / self._ratioX, self.height() - self._bezierList[self._bezierIdx][1] / self._ratioY)
            simPainter.rotate(- self._bezierList[self._bezierIdx][2] * 180 / pi)
            simPainter.drawPolyline(QPolygon(arrowPts))
            simPainter.drawEllipse(-2, -2, 4, 4)
        else:
            self._bezierIdx = 0
    
    def __drawLinePath(self):
        if (self._endIdx >= 1):
            linePathPainter = QPainter(self)
            pathPen = QPen(Qt.yellow, 1)
            linePathPainter.setPen(pathPen)
            for i in range(self._endIdx - 1):
                linePathPainter.drawLine(self._endList[i][0] / self._ratioX, self.height() - self._endList[i][1] / self._ratioY, self._endList[i + 1][0] / self._ratioX, self.height() - self._endList[i + 1][1] / self._ratioY)
            if self._stateMachine == 0:
                pathPen = QPen(Qt.yellow, 0.5)
                linePathPainter.setPen(pathPen)
            linePathPainter.drawLine(self._endList[self._endIdx - 1][0] / self._ratioX, self.height() - self._endList[self._endIdx - 1][1] / self._ratioY, self._endList[self._endIdx][0] / self._ratioX, self.height() - self._endList[self._endIdx][1] / self._ratioY)

    def __drawBezierPath(self):
        bezierPathPainter = QPainter(self)
        pathPen = QPen(Qt.green, 2)
        bezierPathPainter.setPen(pathPen)
        for i in range(self._sampling * int(self._endIdx / self._degree) - 1):
            bezierPathPainter.drawLine(self._bezierList[i][0] / self._ratioX, self.height() - self._bezierList[i][1] / self._ratioY, self._bezierList[i + 1][0] / self._ratioX, self.height() - self._bezierList[i + 1][1] / self._ratioY)
    
    def __drawText(self):
        textPainter = QPainter(self)
        textPainter.setPen(Qt.white)
        textPainter.setFont(QFont('等线', 10))
        textPainter.drawText(QPoint(EditorWindow.MARGIN[0] / self._ratioX, EditorWindow.MARGIN[1] / self._ratioY +  20), "模式: %d阶贝塞尔曲线" % self._degree)
        textPainter.drawText(QPoint(EditorWindow.MARGIN[0] / self._ratioX, EditorWindow.MARGIN[1] / self._ratioY +  40), "段数: %d" % int((self._endIdx - 1) / self._degree))
        textPainter.drawText(QPoint(EditorWindow.MARGIN[0] / self._ratioX, EditorWindow.MARGIN[1] / self._ratioY +  60), "点数: %d => %d" % (self._endIdx, self._sampling * int(self._endIdx / self._degree)))
        textPainter.drawText(QPoint(EditorWindow.MARGIN[0] / self._ratioX, EditorWindow.MARGIN[1] / self._ratioY +  100), "仿真点")
        textPainter.drawText(QPoint(EditorWindow.MARGIN[0] / self._ratioX, EditorWindow.MARGIN[1] / self._ratioY +  120), "X: %.3f m" % self._bezierList[self._bezierIdx][0])
        textPainter.drawText(QPoint(EditorWindow.MARGIN[0] / self._ratioX, EditorWindow.MARGIN[1] / self._ratioY +  140), "Y: %.3f m" % self._bezierList[self._bezierIdx][1])
        textPainter.drawText(QPoint(EditorWindow.MARGIN[0] / self._ratioX, EditorWindow.MARGIN[1] / self._ratioY +  160), "Z: %.2f deg" % (self._bezierList[self._bezierIdx][2] / pi * 180))
        textPainter.drawText(QPoint(EditorWindow.MARGIN[0] / self._ratioX, EditorWindow.MARGIN[1] / self._ratioY +  200), "指针: %s" % ('端点' if self._endIdx % self._degree == 0 else '控制点'))
        textPainter.drawText(QPoint(EditorWindow.MARGIN[0] / self._ratioX, EditorWindow.MARGIN[1] / self._ratioY +  220), "X: %.3f m" % self._endList[self._endIdx][0])
        textPainter.drawText(QPoint(EditorWindow.MARGIN[0] / self._ratioX, EditorWindow.MARGIN[1] / self._ratioY +  240), "Y: %.3f m" % self._endList[self._endIdx][1])
        textPainter.drawText(QPoint(EditorWindow.MARGIN[0] / self._ratioX, EditorWindow.MARGIN[1] / self._ratioY +  260), "Z: %.2f deg" % (self._endList[self._endIdx][2] / pi * 180))

if __name__=='__main__':
    editor = Editor(3, 500)
