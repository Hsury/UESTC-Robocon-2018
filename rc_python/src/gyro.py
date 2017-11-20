from math import sin, cos, pi, sqrt
import serial
import threading

class Gyro():
    '''UESTC 2018 Robocon Team
    Gyro Driver Package
    '''
    def __init__(self, port='/dev/gyro', baudrate=921600):
        self._port = port
        self._baudrate = baudrate
        self._data = [0] * 3
        self._count = 0
        try:
            self._serial = serial.Serial(self._port, self._baudrate)
            queryThd = threading.Thread(target=self.__query)
            queryThd.setDaemon(True)
            queryThd.start()
        except:
            pass
    
    def __query(self):
        while self._serial.is_open:
            self._serial.reset_input_buffer()
            self._serial.readline()
            buffer = self._serial.readline().decode()[:-1].split(' ')
            self._data = [int(buffer[1]) / 1E4, \
                          int(buffer[2]) / 1E4, \
                          float(buffer[0]) * pi / 180]
            self._count += 1
    
    def test(self):
        from time import sleep
        print('Testing Gyro...')
        print('  Port: {}'.format(self._port))
        print('  Baudrate: {}'.format(self._baudrate))
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
        return self._data

if __name__=='__main__':
    gyro = Gyro('/dev/gyro', 921600)
    gyro.test()
