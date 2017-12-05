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
    def __init__(self, filename='rc_trace.txt', interval=0.001):
        self._filename = filename
        self._interval = interval
        self._dataDir = os.path.dirname(os.getcwd()) + os.sep + 'data'
        self.__loadPath()
        self.__show()
    
    def __loadPath(self):
        self._position = np.empty(shape=[0, 3])
        self._goal = np.empty(shape=[0, 3])
        self._idealSpeed = np.empty(shape=[0, 3])
        self._idealSpeedMecanum = np.empty(shape=[0, 4])
        self._idealSpeedOmni = np.empty(shape=[0, 3])
        self._realSpeed = np.empty(shape=[0, 3])
        self._realSpeedMecanum = np.empty(shape=[0, 4])
        self._realSpeedOmni = np.empty(shape=[0, 3])
        self._idealAccel = np.empty(shape=[0, 3])
        self._idealAccelMecanum = np.empty(shape=[0, 4])
        self._idealAccelOmni = np.empty(shape=[0, 3])
        self._realAccel = np.empty(shape=[0, 3])
        self._realAccelMecanum = np.empty(shape=[0, 4])
        self._realAccelOmni = np.empty(shape=[0, 3])
        try:
            with open(self._dataDir + os.sep + self._filename, 'r') as fobj:
                for eachline in fobj:
                    buffer = np.array([[float(data) for data in eachline.split(' ')]])
                    self._position = np.concatenate((self._position, buffer[:, 0: 3]))
                    self._goal = np.concatenate((self._goal, buffer[:, 3: 6]))
                    self._idealSpeed = np.concatenate((self._idealSpeed, buffer[:, 6: 9]))
                    self._idealSpeedMecanum = np.concatenate((self._idealSpeedMecanum, np.array([Mecanum.resolve(self._idealSpeed[-1, 0], self._idealSpeed[-1, 1], self._idealSpeed[-1, 2], self._goal[-1, 2])])))
                    self._idealSpeedOmni = np.concatenate((self._idealSpeedOmni, np.array([Omni.resolve(self._idealSpeed[-1, 0], self._idealSpeed[-1, 1], self._idealSpeed[-1, 2], self._goal[-1, 2])])))
                    if (len(self._position) >= 2):
                        self._realSpeed = np.concatenate((self._realSpeed, np.array([[(self._position[-1, i] - self._position[-2, i]) / self._interval for i in range(3)]])))
                        self._realSpeedMecanum = np.concatenate((self._realSpeedMecanum, np.array([Mecanum.resolve(self._realSpeed[-1, 0], self._realSpeed[-1, 1], self._realSpeed[-1, 2], self._position[-1, 2])])))
                        self._realSpeedOmni = np.concatenate((self._realSpeedOmni, np.array([Omni.resolve(self._realSpeed[-1, 0], self._realSpeed[-1, 1], self._realSpeed[-1, 2], self._position[-1, 2])])))
                        self._idealAccel = np.concatenate((self._idealAccel, np.array([[(self._idealSpeed[-1, i] - self._idealSpeed[-2, i]) / self._interval for i in range(3)]])))
                        self._idealAccelMecanum = np.concatenate((self._idealAccelMecanum, np.array([[(self._idealSpeedMecanum[-1, i] - self._idealSpeedMecanum[-2, i]) / self._interval for i in range(4)]])))
                        self._idealAccelOmni = np.concatenate((self._idealAccelOmni, np.array([[(self._idealSpeedOmni[-1, i] - self._idealSpeedOmni[-2, i]) / self._interval for i in range(3)]])))
                        if (len(self._position) >= 3):
                            self._realAccel = np.concatenate((self._realAccel, np.array([[(self._realSpeed[-1, i] - self._realSpeed[-2, i]) / self._interval for i in range(3)]])))
                            self._realAccelMecanum = np.concatenate((self._realAccelMecanum, np.array([[(self._realSpeedMecanum[-1, i] - self._realSpeedMecanum[-2, i]) / self._interval for i in range(4)]])))
                            self._realAccelOmni = np.concatenate((self._realAccelOmni, np.array([[(self._realSpeedOmni[-1, i] - self._realSpeedOmni[-2, i]) / self._interval for i in range(3)]])))
        except:
            pass
    
    def __show(self):
        tPos = np.arange(len(self._position))
        tSpd = np.arange(len(self._realSpeed))
        tAcc = np.arange(len(self._realAccel))
        plt.figure(num='场地平面图')
        plt.plot(self._goal[:, 0], self._goal[:, 1], 'r', self._position[:, 0], self._position[:, 1], 'b')
        plt.figure(num='整车电机曲线')
        plt.subplot(321); plt.plot(tSpd, self._realSpeed[:, 0]); plt.ylabel('m/s'); plt.title('X-axis Velocity')
        plt.subplot(322); plt.plot(tAcc, self._realAccel[:, 0]); plt.ylabel('m/s^2'); plt.title('X-axis Acceleration')
        plt.subplot(323); plt.plot(tSpd, self._realSpeed[:, 1]); plt.ylabel('m/s'); plt.title('Y-axis Velocity')
        plt.subplot(324); plt.plot(tAcc, self._realAccel[:, 1]); plt.ylabel('m/s^2'); plt.title('Y-axis Acceleration')
        plt.subplot(325); plt.plot(tSpd, self._realSpeed[:, 2]); plt.ylabel('rad/s'); plt.title('Z-axis Velocity')
        plt.subplot(326); plt.plot(tAcc, self._realAccel[:, 2]); plt.ylabel('rad/s^2'); plt.title('Z-axis Acceleration')
        plt.figure(num='麦克纳姆轮电机曲线')
        plt.subplot(421); plt.plot(tSpd, self._realSpeedMecanum[:, 0]); plt.ylabel('cnt/s'); plt.title('Left Front Velocity')
        plt.subplot(422); plt.plot(tAcc, self._realAccelMecanum[:, 0]); plt.ylabel('cnt/s^2'); plt.title('Left Front Acceleration')
        plt.subplot(423); plt.plot(tSpd, self._realSpeedMecanum[:, 1]); plt.ylabel('cnt/s'); plt.title('Left Rear Velocity')
        plt.subplot(424); plt.plot(tAcc, self._realAccelMecanum[:, 1]); plt.ylabel('cnt/s^2'); plt.title('Left Rear Acceleration')
        plt.subplot(425); plt.plot(tSpd, self._realSpeedMecanum[:, 2]); plt.ylabel('cnt/s'); plt.title('Right Rear Velocity')
        plt.subplot(426); plt.plot(tAcc, self._realAccelMecanum[:, 2]); plt.ylabel('cnt/s^2'); plt.title('Right Rear Acceleration')
        plt.subplot(427); plt.plot(tSpd, self._realSpeedMecanum[:, 3]); plt.ylabel('cnt/s'); plt.title('Right Front Velocity')
        plt.subplot(428); plt.plot(tAcc, self._realAccelMecanum[:, 3]); plt.ylabel('cnt/s^2'); plt.title('Right Front Acceleration')
        plt.figure(num='全向轮电机曲线')
        plt.subplot(321); plt.plot(tSpd, self._realSpeedOmni[:, 0]); plt.ylabel('cnt/s'); plt.title('Head Velocity')
        plt.subplot(322); plt.plot(tAcc, self._realAccelOmni[:, 0]); plt.ylabel('cnt/s^2'); plt.title('Head Acceleration')
        plt.subplot(323); plt.plot(tSpd, self._realSpeedOmni[:, 1]); plt.ylabel('cnt/s'); plt.title('Left Velocity')
        plt.subplot(324); plt.plot(tAcc, self._realAccelOmni[:, 1]); plt.ylabel('cnt/s^2'); plt.title('Left Acceleration')
        plt.subplot(325); plt.plot(tSpd, self._realSpeedOmni[:, 2]); plt.ylabel('cnt/s'); plt.title('Right Velocity')
        plt.subplot(326); plt.plot(tAcc, self._realAccelOmni[:, 2]); plt.ylabel('cnt/s^2'); plt.title('Right Acceleration')
        plt.show()

if __name__=='__main__':
    plot = Plot()
