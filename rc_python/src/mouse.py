from math import sin, cos, pi, sqrt
from evdev import InputDevice, categorize, ecodes
import threading

class Mouse():
    '''UESTC 2018 Robocon Team
    Mouse Package
    '''
    def __init__(self, merge, port='/dev/input/event3'):
        self._merge = merge
        self._port = port
        self._x = 0
        self._y = 0
        try:
            self._dev = InputDevice(self._port)
            thd = threading.Thread(target = self.__mouseEvent)
            thd.setDaemon(True)
            thd.start()
        except:
            pass
    
    def __mouseEvent(self):
        for event in self._dev.read_loop():
            if event.type == ecodes.EV_REL:
                if event.code == ecodes.REL_X:
                    self._x += event.value * cos(self._merge.data[2])
                    self._y += event.value * sin(self._merge.data[2])
                if event.code == ecodes.REL_Y:
                    self._x += event.value * sin(self._merge.data[2])
                    self._y -= event.value * cos(self._merge.data[2])
    
    def test(self):
        print(self._dev)
    
    @property
    def data(self):
        return [self._x, self._y]
