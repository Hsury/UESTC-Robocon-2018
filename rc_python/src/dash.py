from math import sin, cos, pi, sqrt
import threading

class Dash():
    '''UESTC 2018 Robocon Team
    Dash Package
    '''
    POSITION_MODE = 0
    SPEED_MODE = 1

    def __init__(self, base, merge):
        self._base = base
        self._merge = merge
        self._mode = Dash.POSITION_MODE
        self._goal = [0] * 3
        self._dist = [0] * 3
        self._speed = [0] * 3
        self.lock()
        sendThd = threading.Thread(target=self.__send)
        sendThd.setDaemon(True)
        sendThd.start()
    
    def __send(self):
        from time import sleep
        while True:
            if not self._lock:
                if self.mode == Dash.POSITION_MODE:
                    self.__resolve(self._merge.data, self._goal)
                #self._speed[0] = self.__limiter(self._speed[0], -2, 2)
                #self._speed[1] = self.__limiter(self._speed[1], -2, 2)
                #self._speed[2] = self.__limiter(self._speed[2], - pi / 2, pi / 2)
                self._base.go(self._speed[0], self._speed[1], self._speed[2], self._merge.data[2])
            sleep(0.005)
    
    def __resolve(self, position, goal):
        for idx in range(3):
            self._dist[idx] = goal[idx] - position[idx]
        while self._dist[2] <= - pi:
            self._dist[2] += 2 * pi
        while self._dist[2] > pi:
            self._dist[2] -= 2 * pi
        # Under Construction
        _safeDist = 1
        _maxSpd = 3
        _linearCoefP = 1
        _angularCoefP = 1
        self._resDist = sqrt(self._dist[0] ** 2 + self._dist[1] ** 2)
        if self._resDist > _safeDist:
            self._speed[0] = self._dist[0] * _maxSpd / self._resDist
            self._speed[1] = self._dist[1] * _maxSpd / self._resDist
        else:
            self._speed[0] = self._dist[0] * _linearCoefP
            self._speed[1] = self._dist[1] * _linearCoefP
        self._speed[2] = self._dist[2] * _angularCoefP

    def __limiter(self, value, lower, upper):
        if value < lower:
            return lower
        elif value > upper:
            return upper
        else:
            return value
    
    def lock(self):
        self._lock = True
        self._goal = self._merge.data[:]
        self._speed = [0] * 3
        self._base.stop()
    
    def unlock(self):
        self._lock = False
    
    def to(self, x, y, z):
        self._mode = Dash.POSITION_MODE
        self._goal = [x, y, z]

    def at(self, x, y, z):
        self._mode = Dash.SPEED_MODE
        self._speed = [x, y, z]
    
    @property
    def locked(self):
        return self._lock

    @property
    def mode(self):
        return self._mode

    @property
    def position(self):
        return self._merge.data

    @property
    def goal(self):
        return self._goal

    @property
    def dist(self):
        return self._dist

    @property
    def speed(self):
        return self._speed
