from math import sin, cos, pi, sqrt
from elmo import Elmo

class Mecanum():
    '''UESTC 2018 Robocon Team
    Mecanum Driver Package
    '''
    WHEEL_RADIUS = 0.075
    SIDE_LENGTH = 0.75
    REDUCTION_RATIO = 11.39 / 2

    def __init__(self, leftFrontPort='/dev/elmo0', leftRearPort='/dev/elmo1', rightFrontPort='/dev/elmo2', rightRearPort='/dev/elmo3', baudrate=19200):
        self._port = [leftFrontPort, leftRearPort, rightFrontPort, rightRearPort]
        self._baudrate = baudrate
        self._elmoList = [Elmo(self._port[0], self._baudrate), \
                          Elmo(self._port[1], self._baudrate), \
                          Elmo(self._port[2], self._baudrate), \
                          Elmo(self._port[3], self._baudrate)]
        self._speedList = [0] * 4
    
    def test(self):
        from time import sleep
        print('Testing Omni...')
        print('  Left Front Port: {}'.format(self._port[0]))
        print('  Left Rear Port: {}'.format(self._port[1]))
        print('  Right Front Port: {}'.format(self._port[2]))
        print('  Right Rear Port: {}'.format(self._port[3]))
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
    
    @classmethod
    def resolve(cls, vX=0, vY=0, wZ=0, orient=0):
        w2v = lambda w: w * Mecanum.SIDE_LENGTH * sqrt(2) / 2
        v2jv = lambda v: int(v / (2 * pi * Mecanum.WHEEL_RADIUS) * Mecanum.REDUCTION_RATIO * 2000)
        return [- v2jv((vX * sin(pi / 4 - orient) + vY * cos(pi / 4 - orient) - w2v(wZ)) * sqrt(2)), \
                v2jv((vX * cos(pi / 4 - orient) - vY * sin(pi / 4 - orient) + w2v(wZ)) * sqrt(2)), \
                v2jv((vX * sin(pi / 4 - orient) + vY * cos(pi / 4 - orient) + w2v(wZ)) * sqrt(2)), \
                - v2jv((vX * cos(pi / 4 - orient) - vY * sin(pi / 4 - orient) - w2v(wZ)) * sqrt(2))]
    
    def go(self, vX=0, vY=0, wZ=0, orient=0):
        self._speedList = self.resolve(vX, vY, wZ, orient + pi)
        for idx in range(4):
            self._elmoList[idx].speed = self._speedList[idx]
        for idx in range(4):
            self._elmoList[idx].begin()
    
    def stop(self):
        self.go()
    
    def enable(self):
        for idx in range(4):
            self._elmoList[idx].enable()
    
    def disable(self):
        for idx in range(4):
            self._elmoList[idx].disable()

if __name__=='__main__':
    mecanum = Mecanum('/dev/elmo0', '/dev/elmo1', '/dev/elmo2', '/dev/elmo3', 19200)
    mecanum.test()
