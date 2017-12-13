from math import sin, cos, pi, sqrt
from time import sleep
import threading

class Flow():
    '''UESTC 2018 Robocon Team
    Flow Package
    '''

    def __init__(self, dash, debug=True):
        self._route = {
            'Self Test': {
                'id': 0,
                'func': self.__selftest
            },
            'From ARSZ To TZ0': {
                'id': 1,
                'func': self.__arsz2tz0
            },
            'From TZ0 To TZ1': {
                'id': 2,
                'func': self.__tz02tz1
            }
        }
        self._dash = dash
        self._debug = debug
        self._permission = True
        self._entrance = None
        thd = threading.Thread(target=self.__brain)
        thd.setDaemon(True)
        thd.start()
    
    def __brain(self):
        self._entrance = 'Self Test'
        while self._entrance != None:
            if self._debug:
                self._permission = False
                while not self._permission:
                    sleep(0.005)
            self._entrance = self._route[self._entrance]['func']()
    
    def go(self):
        self._permission = True
    
    @property
    def busy(self):
        return self._permission

    @property
    def info(self):
        return {'id': self._route[self._entrance]['id'],
                'task': self._entrance}
    
    def __selftest(self):
        print('===== Self Test Begin =====')
        sleep(1)
        print(self._dash.position)
        self._dash.lock()
        self._dash._merge.swift(0.5, 7.5)
        print('===== Self Test End =====')
        return 'From ARSZ To TZ0'

    def __arsz2tz0(self):
        self._dash.to(1.6, 3, 0)
        self._dash.unlock()
        return 'From TZ0 To TZ1'

    def __tz02tz1(self):
        self._dash.to(3.8, 3, 0)
        self._dash.unlock()
        return
