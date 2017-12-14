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
                'func': self.__selfTest
            },
            'Param Init': {
                'id': 1,
                'func': self.__paramInit
            },
            'From ARSZ To TZA': {
                'id': 2,
                'func': self.__ARSZ2TZA
            },
            'From TZA To TZ1': {
                'id': 3,
                'func': self.__TZA2TZ1
            },
            'Wait For Ball': {
                'id': 4,
                'func': self.__waitForBall
            },
            'From TZ1 To TZA': {
                'id': 5,
                'func': self.__TZ12TZA
            },
            'From TZA To TZB': {
                'id': 6,
                'func': self.__TZA2TZB
            },
            'From TZB To TZ2': {
                'id': 7,
                'func': self.__TZB2TZ2
            },
            'From TZ2 To TZ3': {
                'id': 8,
                'func': self.__TZ22TZ3
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
            print('========== ID {}, {} =========='.format(self._route[self._entrance]['id'], self._entrance))
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
    
    # ID 0, Self Test
    def __selfTest(self):
        self._dash.lock()
        print('Angle: {}'.format(self._dash.position[2] * 180 / pi))
        return 'Param Init'

    # ID 1, Param Init
    def __paramInit(self):
        self._dash._merge.swift(0.6, 7.5)
        print('Swift to: {}'.format(self._dash.position))
        return 'From ARSZ To TZA'

    # ID 2, From ARSZ To TZA
    def __ARSZ2TZA(self):
        self._dash.to(2, 3, 0)
        self._dash.unlock()
        while self._dash.resDist >= 0.2:
            sleep(0.005)
        print('Arrive with resultant distance: {}'.format(self._dash.resDist))
        return 'From TZA To TZ1'

    # ID 3, From TZA To TZ1
    def __TZA2TZ1(self):
        self._dash.to(3.8, 3, 0)
        while self._dash.resDist >= 0.05:
            sleep(0.005)
        print('Arrive with resultant distance: {}'.format(self._dash.resDist))
        return 'Wait For Ball'

    # ID 4, Wait For Ball
    def __waitForBall(self):
        print('Wait for 2 second')
        sleep(2)
        print('Done')
        return 'From TZ1 To TZA'

    # ID 5, From TZ1 To TZA
    def __TZ12TZA(self):
        self._dash.to(2, 3, 0)
        while self._dash.resDist >= 0.2:
            sleep(0.005)
        print('Arrive with resultant distance: {}'.format(self._dash.resDist))
        return 'From TZA To TZB'

    # ID 6, From TZA To TZB
    def __TZA2TZB(self):
        self._dash.to(2, 1, 0)
        while self._dash.resDist >= 0.2:
            sleep(0.005)
        print('Arrive with resultant distance: {}'.format(self._dash.resDist))
        return 'From TZB To TZ2'

    # ID 7, From TZB To TZ2
    def __TZB2TZ2(self):
        self._dash.to(3.8, 1, 0)
        while self._dash.resDist >= 0.05:
            sleep(0.005)
        print('Arrive with resultant distance: {}'.format(self._dash.resDist))
        return 'From TZ2 To TZ3'

    # ID 8, From TZ2 To TZ3
    def __TZ22TZ3(self):
        self._dash.to(7, 1, 0)
        while self._dash.resDist >= 0.05:
            sleep(0.005)
        print('Arrive with resultant distance: {}'.format(self._dash.resDist))
        return None
