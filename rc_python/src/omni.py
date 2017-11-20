from math import sin, cos, pi, sqrt
from elmo import Elmo

class Omni():
    '''UESTC 2018 Robocon Team
    Omni Driver Package
    '''
    WHEEL_RADIUS = 0.078
    SIDE_LENGTH = 1.01
    REDUCTION_RATIO = 19

    def __init__(self, headPort='/dev/elmo0', leftPort='/dev/elmo1', rightPort='/dev/elmo2', baudrate=19200):
        self._port = [headPort, leftPort, rightPort]
        self._baudrate = baudrate
        self._elmoList = [Elmo(self._port[0], self._baudrate), \
                          Elmo(self._port[1], self._baudrate), \
                          Elmo(self._port[2], self._baudrate)]
        self._speedList = [0] * 3
    
    def test(self):
        from time import sleep
        print('Testing Omni...')
        print('  Head Port: {}'.format(self._port[0]))
        print('  Left Port: {}'.format(self._port[1]))
        print('  Right Port: {}'.format(self._port[2]))
        print('  Baudrate: {}'.format(self._baudrate))
        print('  Step 1/4: Get serial number: {}'.format([eachElmo.sn for eachElmo in self._elmoList]))
        print('  Step 2/4: Anticlockwise')
        self.go(wZ=pi / 4)
        sleep(2)
        print('  Step 3/4: Break')
        self.go()
        sleep(1)
        print('  Step 4/4: Clockwise')
        self.go(wZ=- pi / 4)
        sleep(2)
        self.go()
        print('Done')
    
    def resolve(self, vX=0, vY=0, wZ=0, orient=0):
        w2v = lambda w: w * Omni.SIDE_LENGTH / sqrt(3)
        v2jv = lambda v: int(v / (2 * pi * Omni.WHEEL_RADIUS) * Omni.REDUCTION_RATIO * 2000)
        return [v2jv(- vX * cos(orient) - vY * sin(orient) + w2v(wZ)), \
                v2jv(vX * sin(pi / 6 + orient) - vY * cos(pi / 6 + orient) + w2v(wZ)), \
                v2jv(vX * sin(pi / 6 - orient) + vY * cos(pi / 6 - orient) + w2v(wZ))]
    
    def go(self, vX=0, vY=0, wZ=0, orient=0):
        self._speedList = self.resolve(vX, vY, wZ, orient)
        for idx in range(3):
            self._elmoList[idx].speed = self._speedList[idx]
        for idx in range(3):
            self._elmoList[idx].begin()
    
    def stop(self):
        self.go()
    
    def enable(self):
        for idx in range(3):
            self._elmoList[idx].enable()
    
    def disable(self):
        for idx in range(3):
            self._elmoList[idx].disable()

if __name__=='__main__':
    omni = Omni('/dev/elmo0', '/dev/elmo1', '/dev/elmo2', 19200)
    omni.test()
