from math import sin, cos, pi, sqrt
import threading

class Dash():
    '''UESTC 2018 Robocon Team
    Dash Package
    '''
    POSITION_MODE = 0
    INCREMENT_MODE = 1
    SPEED_MODE = 2
    TEST_MODE = 3

    SAFE_DIST = 0.5
    MAX_SPEED = 3
    COEF_LINEAR = 1
    COEF_ANGULAR = 1

    def __init__(self, base, merge):
        self._base = base
        self._merge = merge
        self._mode = Dash.POSITION_MODE
        self._goal = [0] * 3
        self._dist = [0] * 3
        self._speed = [0] * 3
        self._resDist = 0
        self._resSpeed = 0
        self.lock()
        sendThd = threading.Thread(target=self.__send)
        sendThd.setDaemon(True)
        sendThd.start()
    
    def __send(self):
        from time import sleep
        while True:
            if not self._lock:
                if self.mode == Dash.POSITION_MODE or self.mode == Dash.INCREMENT_MODE:
                    self.__resolve(self._merge.data, self._goal)
                #Cui's Dash Mode
                if self.mode == Dash.TEST_MODE:
                    self._cuiDash.update(self._merge.data[0], self._merge.data[1], self._merge.data[2])
                    self._speed = self._cuiDash.resolve()
                #self._speed[0] = self.__limiter(self._speed[0], -2, 2)
                #self._speed[1] = self.__limiter(self._speed[1], -2, 2)
                #self._speed[2] = self.__limiter(self._speed[2], - pi / 2, pi / 2)
                self._base.go(self._speed[0], self._speed[1], self._speed[2], self._merge.data[2])
            sleep(0.001)
    
    def __resolve(self, position, goal):
        for idx in range(3):
            self._dist[idx] = goal[idx] - position[idx]
        while self._dist[2] <= - pi:
            self._dist[2] += 2 * pi
        while self._dist[2] > pi:
            self._dist[2] -= 2 * pi
        self._resDist = sqrt(self._dist[0] ** 2 + self._dist[1] ** 2)
        if self._resDist > Dash.SAFE_DIST:
            self._speed[0] = self._dist[0] * Dash.MAX_SPEED / self._resDist
            self._speed[1] = self._dist[1] * Dash.MAX_SPEED / self._resDist
        else:
            self._speed[0] = self._dist[0] * Dash.COEF_LINEAR
            self._speed[1] = self._dist[1] * Dash.COEF_LINEAR
        self._resSpeed = sqrt(self._speed[0] ** 2 + self._speed[1] ** 2)
        self._speed[2] = self._dist[2] * Dash.COEF_ANGULAR

    def __limiter(self, value, lower, upper):
        if value < lower:
            return lower
        elif value > upper:
            return upper
        else:
            return value
    
    def lock(self):
        self._lock = True
        self._speed = [0] * 3
        self._base.stop()
    
    def unlock(self):
        self._lock = False
    
    def to(self, x, y, z):
        self._mode = Dash.POSITION_MODE
        self._goal = [x, y, z]
    
    def by(self, x, y, z):
        self._mode = Dash.INCREMENT_MODE
        self._goal = [self._merge.data[0] + x, self._merge.data[1] + y, self._merge.data[2] + z]

    def at(self, x, y, z):
        self._mode = Dash.SPEED_MODE
        self._speed = [x, y, z]
    
    def cuiInit(self):
        from cuiDash.wrapper import Wrapper
        self._cuiDash = Wrapper()
    
    def cuiTo(self, x, y, z):
        self._mode = Dash.TEST_MODE
        #self._cuiDash.setGoal(x, y, z)
    
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

    @property
    def resDist(self):
        return self._resDist

    @property
    def resSpeed(self):
        return self._resSpeed
