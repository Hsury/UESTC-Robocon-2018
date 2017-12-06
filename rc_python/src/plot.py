from math import sin, cos, pi, sqrt
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
from mecanum import Mecanum
from omni import Omni

class Plot():
    '''UESTC 2018 Robocon Team
    Plot Package
    '''
    def __init__(self, filename='rc_trace.txt', interval=0.01):
        self._filename = filename
        self._interval = interval
        self._dataDir = os.path.dirname(os.getcwd()) + os.sep + 'data'
        self.__loadPath()
        self.__show()
    
    def __loadPath(self):
        self._position = np.empty(shape=[0, 3])
        self._goal = np.empty(shape=[0, 3])
        self._speed = np.empty(shape=[0, 3])
        self._accel = np.empty(shape=[0, 3])
        self._mecanumSpeed = np.empty(shape=[0, 4])
        self._mecanumAccel = np.empty(shape=[0, 4])
        self._omniSpeed = np.empty(shape=[0, 3])
        self._omniAccel = np.empty(shape=[0, 3])
        try:
            with open(self._dataDir + os.sep + self._filename, 'r') as fobj:
                for eachline in fobj:
                    buffer = np.array([[float(data) for data in eachline.split(' ')]])
                    self._position = np.concatenate((self._position, buffer[:, 0: 3]))
                    self._goal = np.concatenate((self._goal, buffer[:, 3: 6]))
                    self._speed = np.concatenate((self._speed, buffer[:, 6: 9]))
                    self._mecanumSpeed = np.concatenate((self._mecanumSpeed, np.array([Mecanum.resolve(self._speed[-1, 0], self._speed[-1, 1], self._speed[-1, 2], self._position[-1, 2])])))
                    self._omniSpeed = np.concatenate((self._omniSpeed, np.array([Omni.resolve(self._speed[-1, 0], self._speed[-1, 1], self._speed[-1, 2], self._position[-1, 2])])))
                    if (len(self._position) >= 2):
                        self._accel = np.concatenate((self._accel, np.array([[(self._speed[-1, i] - self._speed[-2, i]) / self._interval for i in range(3)]])))
                        self._mecanumAccel = np.concatenate((self._mecanumAccel, np.array([[(self._mecanumSpeed[-1, i] - self._mecanumSpeed[-2, i]) / self._interval for i in range(4)]])))
                        self._omniAccel = np.concatenate((self._omniAccel, np.array([[(self._omniSpeed[-1, i] - self._omniSpeed[-2, i]) / self._interval for i in range(3)]])))
        except:
            pass
    
    def __show(self):
        tPos = np.arange(len(self._position)) * self._interval
        tSpd = np.arange(len(self._speed)) * self._interval
        tAcc = np.arange(len(self._accel)) * self._interval
        fig = plt.figure(num='位姿')
        gs = GridSpec(4, 2)
        plt.subplot(gs[0: 2, 0])
        plt.plot(np.insert(self._goal[:, 0], 0, self._position[0, 0]), np.insert(self._goal[:, 1], 0, self._position[0, 1]), '--', label='Ideal')
        plt.plot(self._position[:, 0], self._position[:, 1], label='Real')
        plt.legend()
        plt.xlabel('X(m)')
        plt.ylabel('Y(m)')
        plt.title('Position')
        plt.axis('equal')
        ax = fig.add_subplot(gs[0: 2, 1], projection='3d')
        ax.plot(np.insert(self._goal[:, 0], 0, self._position[0, 0]), np.insert(self._goal[:, 1], 0, self._position[0, 1]), np.insert(self._goal[:, 2], 0, self._position[0, 2]), '--', label='Ideal')
        ax.plot(self._position[:, 0], self._position[:, 1], self._position[:, 2], label='Real')
        ax.legend()
        ax.set_xlabel('X(m)')
        ax.set_ylabel('Y(m)')
        ax.set_zlabel('Z(rad)')
        plt.title('Orientation')
        ax.axis('equal')
        plt.subplot(gs[2: 3, :])
        plt.plot(tPos, self._position[:, 0], label='X')
        plt.plot(tPos, self._position[:, 1], label='Y')
        plt.legend()
        plt.xlabel('Time(s)')
        plt.ylabel('Dist(m)')
        plt.title('X-Y Distance')
        plt.subplot(gs[3: 4, :])
        plt.plot(tPos, self._position[:, 2], label='Z')
        plt.legend()
        plt.xlabel('Time(s)')
        plt.ylabel('Dist(rad)')
        plt.title('Z Distance')
        plt.tight_layout()
        manager = plt.get_current_fig_manager()
        manager.window.showMaximized()
        plt.figure(num='整体速度与加速度')
        gs = GridSpec(4, 2)
        plt.subplot(gs[0: 3, 0])
        plt.plot(tSpd, self._speed[:, 0], label='X')
        plt.plot(tSpd, self._speed[:, 1], label='Y')
        plt.plot(tSpd, np.hypot(self._speed[:, 0], self._speed[:, 1]), label='X-Y')
        plt.legend()
        plt.xlabel('Time(s)')
        plt.ylabel('Vel(m/s)')
        plt.title('X-Y Velocity')
        plt.subplot(gs[0: 3, 1])
        plt.plot(tAcc, self._accel[:, 0], label='X')
        plt.plot(tAcc, self._accel[:, 1], label='Y')
        plt.plot(tAcc, np.hypot(self._accel[:, 0], self._accel[:, 1]), label='X-Y')
        plt.legend()
        plt.xlabel('Time(s)')
        plt.ylabel('Acc(m/s^2)')
        plt.title('X-Y Acceleration')
        plt.subplot(gs[3: 4, 0])
        plt.plot(tSpd, self._speed[:, 2], label='Z')
        plt.legend()
        plt.xlabel('Time(s)')
        plt.ylabel('Vel(rad/s)')
        plt.title('Z Velocity')
        plt.subplot(gs[3: 4, 1])
        plt.plot(tAcc, self._accel[:, 2], label='Z')
        plt.legend()
        plt.xlabel('Time(s)')
        plt.ylabel('Acc(rad/s^2)')
        plt.title('Z Acceleration')
        plt.tight_layout()
        manager = plt.get_current_fig_manager()
        manager.window.showMaximized()
        plt.figure(num='电机速度与加速度')
        plt.subplot(221)
        plt.plot(tSpd, self._mecanumSpeed[:, 0], label='Left Front')
        plt.plot(tSpd, self._mecanumSpeed[:, 1], label='Left Rear')
        plt.plot(tSpd, self._mecanumSpeed[:, 2], label='Right Rear')
        plt.plot(tSpd, self._mecanumSpeed[:, 3], label='Right Front')
        plt.legend()
        plt.xlabel('Time(s)')
        plt.ylabel('Vel(cnt/s)')
        plt.title('Mecanum Velocity')
        plt.subplot(222)
        plt.plot(tAcc, self._mecanumAccel[:, 0], label='Left Front')
        plt.plot(tAcc, self._mecanumAccel[:, 1], label='Left Rear')
        plt.plot(tAcc, self._mecanumAccel[:, 2], label='Right Rear')
        plt.plot(tAcc, self._mecanumAccel[:, 3], label='Right Front')
        plt.legend()
        plt.xlabel('Time(s)')
        plt.ylabel('Vel(cnt/s^2)')
        plt.title('Mecanum Acceleration')
        plt.subplot(223)
        plt.plot(tSpd, self._omniSpeed[:, 0], label='Head')
        plt.plot(tSpd, self._omniSpeed[:, 1], label='Left')
        plt.plot(tSpd, self._omniSpeed[:, 2], label='Right')
        plt.legend()
        plt.xlabel('Time(s)')
        plt.ylabel('Vel(cnt/s)')
        plt.title('Omni Velocity')
        plt.subplot(224)
        plt.plot(tAcc, self._omniAccel[:, 0], label='Head')
        plt.plot(tAcc, self._omniAccel[:, 1], label='Left')
        plt.plot(tAcc, self._omniAccel[:, 2], label='Right')
        plt.legend()
        plt.xlabel('Time(s)')
        plt.ylabel('Vel(cnt/s^2)')
        plt.title('Omni Acceleration')
        plt.tight_layout()
        manager = plt.get_current_fig_manager()
        manager.window.showMaximized()
        plt.show()

if __name__=='__main__':
    plot = Plot()
