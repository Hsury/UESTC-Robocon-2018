from math import sin, cos, pi, sqrt
import os
import threading
import numpy as np

class Feed():
    '''
    UESTC 2018 Robocon Team
    Feed Package
    '''
    def __init__(self, dash, filename='rc_bezier.txt'):
        self._dash = dash
        self._filename = filename
        self._dataDir =  os.getcwd() + os.sep + 'data'  #To Get Parent Directory Use os.path.dirname(os.getcwd())
        self._ctrl = -1
        self._feedList = np.empty(shape=[0, 3])
        self.__loadPath()
    
    def __loadPath(self):
        try:
            with open(self._dataDir + os.sep + self._filename, 'r') as fobj:
                for eachline in fobj:
                    buffer = [float(data) for data in eachline.split(' ')]
                    self._feedList = np.concatenate((self._feedList, np.array([buffer])))
        except:
            pass
    
    def __broadcast(self, interval=0.01, zero=0, step=1):
        from time import sleep
        self._ctrl = 0
        self._dash.unlock()
        for self._idx in range(zero, len(self._feedList), step):
            while self._ctrl == 1:
                sleep(0.005)
            if self._ctrl == 2:
                break
            self._dash.to(self._feedList[self._idx][0], self._feedList[self._idx][1], self._feedList[self._idx][2])
            sleep(interval)
            if self._dash.locked == True or self._dash.mode != self._dash.POSITION_MODE or self._dash.goal != self._feedList[self._idx].tolist():
                self.pause()
        self._dash.lock()
        self._ctrl = -1
    
    def __broadcastTest(self, accTime=1, unifTime=10, decTime=1):   #Unavailable
        from time import sleep
        import time
        millis = lambda: int(round(time.time() * 1000))
        totalTime = accTime + unifTime + decTime
        unifInterval = unifTime / len(self._feedList)
        print('Interval={}, ListLength={}'.format(unifInterval, len(self._feedList)))
        self._ctrl = 0
        #self._dash.unlock()
        self._idx = 0
        startTimestamp = millis()
        while millis() - startTimestamp <= unifTime * 1000:
            while self._ctrl == 1:
                sleep(0.005)
            if self._ctrl == 2:
                break
            realIndex = int((millis() - startTimestamp) // (unifInterval * 1000))
            if realIndex >= len(self._feedList):
                break
            elif self._idx != realIndex:
                self._idx = realIndex
                print('TimeDelta={}, Index={}'.format(millis() - startTimestamp, self._idx))
                deltaX = self._feedList[self._idx][0] - self._dash.position[0]
                deltaY = self._feedList[self._idx][1] - self._dash.position[1]
                deltaZ = self._feedList[self._idx][2] - self._dash.position[2]
                speed = [deltaX / unifInterval, deltaY / unifInterval, deltaZ / unifInterval]
                speed[0] = max(-3, min(3, speed[0]))
                speed[1] = max(-3, min(3, speed[1]))
                speed[2] = max(- pi / 2, min(pi / 2, speed[1]))
                self._dash._base.go(speed[0], speed[1], speed[2], self._dash._merge.data[2])
                print(' ', speed[0], speed[1], speed[2])
        self._dash.to(0, 0, 0)
        self._dash.lock()
        self._ctrl = -1
    
    def play(self, interval=0.01, zero=0, step=1):
        if self._ctrl == -1:
            broadcastThd = threading.Thread(target=self.__broadcast, args=(interval, zero, step))
            broadcastThd.setDaemon(True)
            broadcastThd.start()
    
    def resume(self):
        if self._ctrl != -1:
            self._ctrl = 0
            self._dash.unlock()
    
    def pause(self):
        if self._ctrl != -1:
            self._ctrl = 1
    
    def stop(self):
        if self._ctrl != -1:
            self._ctrl = 2
    
    @property
    def data(self):
        return self._feedList.tolist()

    @property
    def status(self):
        if self._ctrl != -1:
            return (self._idx + 1, self._feedList[self._idx].tolist())
