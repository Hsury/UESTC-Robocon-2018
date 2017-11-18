from math import sin, cos, pi, sqrt
import threading

class Dash():
    '''UESTC 2018 Robocon Team
    Dash Package
    '''
    def __init__(self, omni, merge):
        self._omni = omni
        self._merge = merge
        self._lock = True
        self._goal = [0] * 3
        self._dist = [0] * 3
        self._speed = [0] * 3
        sendThd = threading.Thread(target=self.__send)
        sendThd.setDaemon(True)
        sendThd.start()
    
    def __send(self):
        from time import sleep
        while True:
            self.__solve(self._merge.data, self._goal)
            if not self._lock:
                self._omni.go(self._speed[0], self._speed[1], self._speed[2], self._merge.data[2])
            sleep(0.001)
    
    def __solve(self, position, goal):
        for idx in range(3):
            self._dist[idx] = goal[idx] - position[idx]
        while self._dist[2] <= - pi:
            self._dist[2] += 2 * pi
        while self._dist[2] > pi:
            self._dist[2] -= 2 * pi
        self._speed[0] = self.__limiter(self._dist[0], -2, 2)
        self._speed[1] = self.__limiter(self._dist[1], -2, 2)
        self._speed[2] = self.__limiter(self._dist[2], - pi / 2, pi / 2)

    def __limiter(self, value, lower, upper):
        if value < lower:
            return lower
        elif value > upper:
            return upper
        else:
            return value
    
    def lock(self):
        self._lock = True
        self._omni.stop()
    
    def unlock(self):
        self._lock = False
    
    def discard(self):
        self.lock()
        self._goal = self._merge.data[:]
    
    def to(self, x, y, z):
        self._goal = [x, y, z]

    @property
    def goal(self):
        return self._goal

    @property
    def dist(self):
        return self._dist

    @property
    def speed(self):
        return self._speed
