from math import sin, cos, pi, sqrt
from gyro import Gyro
import threading

class Merge():
    '''
    UESTC 2018 Robocon Team
    Merge Package
    '''
    def __init__(self, gyroPort='/dev/gyro', gyroBaudrate=921600):
        self._gyroPort = gyroPort
        self._gyroBaudrate = gyroBaudrate
        self._gyro = Gyro(self._gyroPort, self._gyroBaudrate)
        self._position = [0] * 3
        self._offset = [0] * 3
        self._count = 0
        queryThd = threading.Thread(target=self.__query)
        queryThd.setDaemon(True)
        queryThd.start()
    
    def __query(self):
        from time import sleep
        while True:
            for idx in range(3):
                self._position[idx] = self._offset[idx] + self._gyro.data[idx]
            while self._position[2] <= - pi:
                self._position[2] += 2 * pi
            while self._position[2] > pi:
                self._position[2] -= 2 * pi
            self._count += 1
            sleep(0.005)
    
    def reset(self):
        self.swift()
    
    def swift(self, x=0, y=0):
        self._offset[0] += x - self._position[0]
        self._offset[1] += y - self._position[1]
    
    def test(self):
        from time import sleep
        print('Testing Merge...')
        count = self._count
        sleep(1)
        print('  Step 1/4: Get 1st data: {}'.format(self.data))
        sleep(1)
        print('  Step 2/4: Get 2nd data: {}'.format(self.data))
        sleep(1)
        print('  Step 3/4: Get 3rd data: {}'.format(self.data))
        sleep(1)
        print('  Step 4/4: Calculate frequency: {}'.format((self._count - count) / 4))
        print('Done')
    
    @property
    def data(self):
        return self._position

if __name__=='__main__':
    merge = Merge('/dev/gyro', 921600)
    merge.test()
