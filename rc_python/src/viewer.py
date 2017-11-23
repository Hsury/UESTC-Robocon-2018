from math import sin, cos, atan, pi, sqrt, fabs
import multiprocessing as mp
import threading
import sys
import os
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class Viewer():
    '''UESTC 2018 Robocon Team
    Viewer
    '''
    def __init__(self, dash):
        self._dash = dash
        try:
            mp.set_start_method('spawn')
        except:
            pass
        self._getGoalQ = mp.Queue()
        self._setGoalQ = mp.Queue()
        self._position = mp.Array('d', [0] * 3)
        self._speed = mp.Array('d', [0] * 3)
        p = mp.Process(target=ViewerWindow, args=(self._getGoalQ, self._setGoalQ, self._position, self._speed))
        p.start()
        proxyThd = threading.Thread(target=self.__proxy, args=(p,))
        proxyThd.setDaemon(True)
        proxyThd.start()
    
    def __proxy(self, p):
        from time import sleep
        while p.is_alive():
            for idx in range(3):
                self._position[idx] = self._dash._merge.data[idx]
            for idx in range(3):
                self._speed[idx] = self._dash.speed[idx]
            try:
                while self._setGoalQ.qsize() != 0:
                    buffer = self._setGoalQ.get(block=False)
                    if buffer[3]:
                        self._dash.to(buffer[0], buffer[1], buffer[2])
                        self._dash.unlock()
                    else:
                        self._dash.lock()
                if self._getGoalQ.qsize() == 0:
                    self._getGoalQ.put(self._dash.goal)
            except:
                pass
            sleep(0.005)

class ViewerWindow(QWidget):
    MAP_SIZE = 800
    ARROW_SIZE = 15
    MARGIN = [1.5, 0.5]
    LINE_ERR = 0.001
    ANGULAR_ERR = 0.5

    def __init__(self, getGoalQ, setGoalQ, position, speed):
        app = QApplication(sys.argv)
        super(ViewerWindow, self).__init__()
        self._getGoalQ = getGoalQ
        self._setGoalQ = setGoalQ
        self._position = position
        self._speed = speed
        self._dataDir = os.path.dirname(os.getcwd()) + os.sep + 'data'
        self._positionList = np.zeros((1, 3))
        self._feedList = np.empty(shape=[0, 3])
        self._cursor = [0] * 3
        self._goal = [0] * 3
        self._stateMachine = 0
        self.__loadPath()
        self.setWindowTitle('UESTC Robocon 2018 - 控制台')
        self.resize(ViewerWindow.MAP_SIZE, ViewerWindow.MAP_SIZE)
        self.setMouseTracking(True)
        self.timer = QTimer()
        self.timer.timeout.connect(self.__timer)
        self.timer.start(10)
        self.show()
        sys.exit(app.exec_())
    
    def paintEvent(self, event):
        self._ratioX = 14 / self.width()
        self._ratioY = 14 / self.height()
        self.__drawMap()
        self.__drawPath()
        self.__drawPoint()
        self.__drawText()
    
    def mouseMoveEvent(self, event):
        if self._stateMachine == 0:
            self._cursor[0] = self._ratioX * event.pos().x()
            self._cursor[1] = self._ratioY * (self.height() - event.pos().y())
        elif self._stateMachine == 1:
            if self._ratioX * event.pos().x() < self._cursor[0]:
                self._cursor[2] = atan((self._ratioY * (self.height() - event.pos().y()) - self._cursor[1]) / (self._ratioX * event.pos().x() - self._cursor[0])) + pi / 2
            elif self._ratioX * event.pos().x() > self._cursor[0]:
                self._cursor[2] = atan((self._ratioY * (self.height() - event.pos().y()) - self._cursor[1]) / (self._ratioX * event.pos().x() - self._cursor[0])) - pi / 2
            else:
                if self._ratioY * (self.height() - event.pos().y()) > self._cursor[1]:
                    self._cursor[2] = 0
                elif self._ratioY * (self.height() - event.pos().y()) < self._cursor[1]:
                    self._cursor[2] = - pi
    
    def mousePressEvent(self, event):
        if self._stateMachine == 0:
            if event.button() == Qt.LeftButton:
                self._stateMachine = 1
            elif event.button() == Qt.RightButton:
                self._cursor[2] = self._goal[2]
                self._setGoalQ.put([0, 0, 0, 0])
        elif self._stateMachine == 1:
            if event.button() == Qt.LeftButton:
                self._goal = self._cursor[0], self._cursor[1], self._cursor[2]
                self._setGoalQ.put([self._goal[0], self._goal[1], self._goal[2], 1])
                self._stateMachine = 0
            elif event.button() == Qt.RightButton:
                self._cursor[2] = self._goal[2]
                self._stateMachine = 0
            self._cursor[0] = self._ratioX * event.pos().x()
            self._cursor[1] = self._ratioY * (self.height() - event.pos().y())
    
    def __loadPath(self):
        try:
            with open(self._dataDir + os.sep + 'rc_bezier.txt', 'r') as fobj:
                for eachline in fobj:
                    buffer = [float(data) for data in eachline.split(' ')]
                    self._feedList = np.concatenate((self._feedList, np.array([buffer])))
        except:
            pass
    
    def __timer(self):
        try:
            while self._getGoalQ.qsize() != 0:
                buffer = self._getGoalQ.get(block=False)
                self._goal = buffer
        except:
            pass
        if fabs(self._positionList[-1][0] - self._position[0]) > ViewerWindow.LINE_ERR or fabs(self._positionList[-1][1] - self._position[1]) > ViewerWindow.LINE_ERR or fabs(self._positionList[-1][2] - self._position[2]) > ViewerWindow.ANGULAR_ERR:
            self._positionList = np.concatenate((self._positionList, np.array([self._position])))
        self.update()
    
    def __drawMap(self):
        mapPainter = QPainter(self)
        pixmap = QPixmap(self._dataDir + os.sep + "map.png")
        mapPainter.drawPixmap(self.rect(), pixmap)
    
    def __drawPoint(self):
        ptPainter = QPainter(self)
        arrowPts = [QPoint(- ViewerWindow.ARROW_SIZE / 2, ViewerWindow.ARROW_SIZE * sqrt(3) / 2), QPoint(0, 0), QPoint(ViewerWindow.ARROW_SIZE / 2, ViewerWindow.ARROW_SIZE * sqrt(3) / 2)]
        ptPainter.save()
        ptPainter.translate(self._goal[0] / self._ratioX, self.height() - self._goal[1] / self._ratioY)
        ptPainter.rotate(- self._goal[2] * 180 / pi)
        arrowPen = QPen(Qt.blue, 2)
        ptPainter.setPen(arrowPen)
        ptPainter.drawPolyline(QPolygon(arrowPts))
        ptPainter.restore()
        ptPainter.save()
        ptPainter.translate(self._positionList[-1][0] / self._ratioX, self.height() - self._positionList[-1][1] / self._ratioY)
        ptPainter.rotate(- self._positionList[-1][2] * 180 / pi)
        arrowPen = QPen(Qt.green, 2)
        ptPainter.setPen(arrowPen)
        ptPainter.drawPolyline(QPolygon(arrowPts))
        ptPainter.restore()
        ptPainter.save()
        ptPainter.translate(self._cursor[0] / self._ratioX, self.height() - self._cursor[1] / self._ratioY)
        ptPainter.rotate(- self._cursor[2] * 180 / pi)
        if self._stateMachine == 0:
            arrowPen = QPen(Qt.red, 2)
        elif self._stateMachine == 1:
            arrowPen = QPen(Qt.magenta, 2)
        ptPainter.setPen(arrowPen)
        ptPainter.drawPolyline(QPolygon(arrowPts))
        ptPainter.restore()
    
    def __drawPath(self):
        pathPainter = QPainter(self)
        pathPen = QPen(Qt.green, 1)
        pathPainter.setPen(pathPen)
        for i in range(self._feedList.shape[0] - 1):
            pathPainter.drawLine(self._feedList[i][0] / self._ratioX, self.height() - self._feedList[i][1] / self._ratioY, self._feedList[i + 1][0] / self._ratioX, self.height() - self._feedList[i + 1][1] / self._ratioY)
        pathPen = QPen(Qt.red, 1)
        pathPainter.setPen(pathPen)
        for i in range(1, self._positionList.shape[0] - 1):
            pathPainter.drawLine(self._positionList[i][0] / self._ratioX, self.height() - self._positionList[i][1] / self._ratioY, self._positionList[i + 1][0] / self._ratioX, self.height() - self._positionList[i + 1][1] / self._ratioY)
        pathPen = QPen(Qt.yellow, 1)
        pathPainter.setPen(pathPen)
        pathPainter.drawLine(self._positionList[-1][0] / self._ratioX, self.height() - self._positionList[-1][1] / self._ratioY, self._goal[0] / self._ratioX, self.height() - self._goal[1] / self._ratioY)
        pathPen = QPen(Qt.yellow, 0.5)
        pathPainter.setPen(pathPen)
        pathPainter.drawLine(self._positionList[-1][0] / self._ratioX, self.height() - self._positionList[-1][1] / self._ratioY, self._cursor[0] / self._ratioX, self.height() - self._cursor[1] / self._ratioY)
    
    def __drawText(self):
        textPainter = QPainter(self)
        textPainter.setPen(Qt.white)
        textPainter.setFont(QFont('等线', 10))
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  20), "实际")
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  40), "X: %.3f m" % self._positionList[-1][0])
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  60), "Y: %.3f m" % self._positionList[-1][1])
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  80), "Z: %.2f deg" % (self._positionList[-1][2] / pi * 180))
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  120), "目的")
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  140), "X: %.3f m" % self._goal[0])
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  160), "Y: %.3f m" % self._goal[1])
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  180), "Z: %.2f deg" % (self._goal[2] / pi * 180))
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  220), "指针")
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  240), "X: %.3f m" % self._cursor[0])
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  260), "Y: %.3f m" % self._cursor[1])
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  280), "Z: %.2f deg" % (self._cursor[2] / pi * 180))
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  320), "速度")
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  340), "X-Y: %.3f m/s" % sqrt(self._speed[0] ** 2 + self._speed[1] ** 2))
        textPainter.drawText(QPoint(ViewerWindow.MARGIN[0] / self._ratioX, ViewerWindow.MARGIN[1] / self._ratioY +  360), "Z: %.2f deg/s" % (self._speed[2] / pi * 180))

if __name__=='__main__':
    from omni import Omni
    from merge import Merge
    from dash import Dash
    omni = Omni()
    merge = Merge()
    dash = Dash(omni, merge)
    viewer = Viewer(dash)
