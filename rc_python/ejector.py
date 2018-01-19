from elmo import Elmo

class Ejector():
    '''
    UESTC 2018 Robocon Team
    Ejector Package
    '''
    def __init__(self, leftPort='COM3', rightPort='COM6', baudrate=19200):
        self._port = [leftPort, rightPort]
        self._baudrate = baudrate
        self._elmoList = [Elmo(self._port[0], self._baudrate), \
                          Elmo(self._port[1], self._baudrate)]
        self._v = 0
    
    def go(self, v=200000):
        self._v = v
        self._elmoList[0].speed = self._v
        self._elmoList[1].speed = - self._v
        self._elmoList[0].begin()
        self._elmoList[1].begin()
    
    def reverse(self):
        self.go(v=- self._v)
    
    def stop(self):
        self.go(v=0)
    
    def release(self):
        self.stop()
        self._elmoList[0].disable()
        self._elmoList[1].disable()
    
    @property
    def speed(self):
        return [self._elmoList[0].speed, self._elmoList[1].speed]
