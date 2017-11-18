from math import sin, cos, pi, sqrt
import sys
import os
import numpy as np

class Feed():
    '''UESTC 2018 Robocon Team
    Feed Package
    '''
    def __init__(self, dash, filename='rc_bezier.txt'):
        self._dash = dash
        self._filename = filename
        self._dataDir = os.path.dirname(os.getcwd()) + os.sep + 'data'
        self._feedList = np.empty(shape=[0, 3])
        self._loadPath()
    
    def _loadPath(self):
        try:
            with open(self._dataDir + os.sep + self._filename, 'r') as fobj:
                for eachline in fobj:
                    buffer = [float(data) for data in eachline.split(' ')]
                    self._feedList = np.concatenate((self._feedList, np.array([buffer])))
        except:
            pass
    
    def broadcast(self, interval=0.1, zero=0, step=1):
        from time import sleep
        self._dash.discard()
        self._dash.unlock()
        for idx in range(zero, len(self._feedList), step):
            self._dash.to(self._feedList[idx][0], self._feedList[idx][1], self._feedList[idx][2])
            print("No.{}, X={:.4f}, Y={:.4f}, Z={:.4f}".format(idx + 1, self._feedList[idx][0], self._feedList[idx][1], self._feedList[idx][2]), end='\r')
            sys.stdout.flush()
            sleep(interval)
        print()
        self._dash.discard()
    
    @property
    def data(self):
        return self._feedList
