from math import sin, cos, pi, sqrt
import os
import numpy as np
import matplotlib.pyplot as plt
from mecanum import Mecanum
from omni import Omni

class Plot():
    '''UESTC 2018 Robocon Team
    Plot Package
    '''
    def __init__(self, filename='rc_bezier.txt'):
        self._filename = filename
        self._dataDir = os.path.dirname(os.getcwd()) + os.sep + 'data'
        self.__loadPath()
        self.__show()
    
    def __loadPath(self):
        self._locList = np.empty(shape=[0, 3])
        self._spdList = np.empty(shape=[0, 3])
        self._spdMecanumList = np.empty(shape=[0, 4])
        self._spdOmniList = np.empty(shape=[0, 3])
        self._accList = np.empty(shape=[0, 3])
        self._accMecanumList = np.empty(shape=[0, 4])
        self._accOmniList = np.empty(shape=[0, 3])
        try:
            with open(self._dataDir + os.sep + self._filename, 'r') as fobj:
                for eachline in fobj:
                    self._locList = np.concatenate((self._locList, np.array([[float(data) for data in eachline.split(' ')]])))
                    if (len(self._locList) >= 2):
                        self._spdList = np.concatenate((self._spdList, np.array([[self._locList[-1, i] - self._locList[-2, i] for i in range(3)]])))
                        self._spdMecanumList = np.concatenate((self._spdMecanumList, np.array([Mecanum.resolve(self._spdList[-1, 0], self._spdList[-1, 1], self._spdList[-1, 2], (self._locList[-1, 2] + self._locList[-2, 2]) / 2)])))
                        self._spdOmniList = np.concatenate((self._spdOmniList, np.array([Omni.resolve(self._spdList[-1, 0], self._spdList[-1, 1], self._spdList[-1, 2], (self._locList[-1, 2] + self._locList[-2, 2]) / 2)])))
                        if (len(self._locList) >= 3):
                            self._accList = np.concatenate((self._accList, np.array([[self._spdList[-1, i] - self._spdList[-2, i] for i in range(3)]])))
                            self._accMecanumList = np.concatenate((self._accMecanumList, np.array([[self._spdMecanumList[-1, i] - self._spdMecanumList[-2, i] for i in range(4)]])))
                            self._accOmniList = np.concatenate((self._accOmniList, np.array([[self._spdOmniList[-1, i] - self._spdOmniList[-2, i] for i in range(3)]])))
        except:
            pass
    
    def __show(self):
        tSpd = np.arange(len(self._spdList))
        tAcc = np.arange(len(self._accList))
        plt.figure(num='整车电机曲线')
        plt.subplot(321); plt.plot(tSpd, self._spdList[:, 0]); plt.ylabel('m/s'); plt.title('X-axis Velocity')
        plt.subplot(322); plt.plot(tAcc, self._accList[:, 0]); plt.ylabel('m/s^2'); plt.title('X-axis Acceleration')
        plt.subplot(323); plt.plot(tSpd, self._spdList[:, 1]); plt.ylabel('m/s'); plt.title('Y-axis Velocity')
        plt.subplot(324); plt.plot(tAcc, self._accList[:, 1]); plt.ylabel('m/s^2'); plt.title('Y-axis Acceleration')
        plt.subplot(325); plt.plot(tSpd, self._spdList[:, 2]); plt.ylabel('rad/s'); plt.title('Z-axis Velocity')
        plt.subplot(326); plt.plot(tAcc, self._accList[:, 2]); plt.ylabel('rad/s^2'); plt.title('Z-axis Acceleration')
        plt.figure(num='麦克纳姆轮电机曲线')
        plt.subplot(421); plt.plot(tSpd, self._spdMecanumList[:, 0]); plt.ylabel('cnt/s'); plt.title('Left Front Velocity')
        plt.subplot(422); plt.plot(tAcc, self._accMecanumList[:, 0]); plt.ylabel('cnt/s^2'); plt.title('Left Front Acceleration')
        plt.subplot(423); plt.plot(tSpd, self._spdMecanumList[:, 1]); plt.ylabel('cnt/s'); plt.title('Left Rear Velocity')
        plt.subplot(424); plt.plot(tAcc, self._accMecanumList[:, 1]); plt.ylabel('cnt/s^2'); plt.title('Left Rear Acceleration')
        plt.subplot(425); plt.plot(tSpd, self._spdMecanumList[:, 2]); plt.ylabel('cnt/s'); plt.title('Right Rear Velocity')
        plt.subplot(426); plt.plot(tAcc, self._accMecanumList[:, 2]); plt.ylabel('cnt/s^2'); plt.title('Right Rear Acceleration')
        plt.subplot(427); plt.plot(tSpd, self._spdMecanumList[:, 3]); plt.ylabel('cnt/s'); plt.title('Right Front Velocity')
        plt.subplot(428); plt.plot(tAcc, self._accMecanumList[:, 3]); plt.ylabel('cnt/s^2'); plt.title('Right Front Acceleration')
        plt.figure(num='全向轮电机曲线')
        plt.subplot(321); plt.plot(tSpd, self._spdOmniList[:, 0]); plt.ylabel('cnt/s'); plt.title('Head Velocity')
        plt.subplot(322); plt.plot(tAcc, self._accOmniList[:, 0]); plt.ylabel('cnt/s^2'); plt.title('Head Acceleration')
        plt.subplot(323); plt.plot(tSpd, self._spdOmniList[:, 1]); plt.ylabel('cnt/s'); plt.title('Left Velocity')
        plt.subplot(324); plt.plot(tAcc, self._accOmniList[:, 1]); plt.ylabel('cnt/s^2'); plt.title('Left Acceleration')
        plt.subplot(325); plt.plot(tSpd, self._spdOmniList[:, 2]); plt.ylabel('cnt/s'); plt.title('Right Velocity')
        plt.subplot(326); plt.plot(tAcc, self._accOmniList[:, 2]); plt.ylabel('cnt/s^2'); plt.title('Right Acceleration')
        plt.show()

if __name__=='__main__':
    plot = Plot()
