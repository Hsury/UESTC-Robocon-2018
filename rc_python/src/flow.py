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
            'Wait For Ball At TZ1': {
                'id': 4,
                'func': self.__waitAtTZ1
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
            'Wait For Ball At TZ2': {
                'id': 8,
                'func': self.__waitAtTZ2
            },
            'From TZ2 To TZ3': {
                'id': 9,
                'func': self.__TZ22TZ3
            },
            'Wait For Ball At TZ3': {
                'id': 10,
                'func': self.__waitAtTZ3
            },
            'From TZ3 To ARSZ': {
                'id': 11,
                'func': self.__TZ32ARSZ
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
        self._dash._merge.swift(0.55, 7.54)
        print('Swift to: {}'.format(self._dash.position))
        return 'From ARSZ To TZA'

    # ID 2, From ARSZ To TZA
    def __ARSZ2TZA(self):
        self._dash.to(1.425, 3.055, 0)
        self._dash.unlock()
        while self._dash.resDist >= 0.2:
            sleep(0.005)
        print('Arrive with resultant distance: {}'.format(self._dash.resDist))
        return 'From TZA To TZ1'

    # ID 3, From TZA To TZ1
    def __TZA2TZ1(self):
        self._dash.to(3.775, 3.055, 0)
        while self._dash.resDist >= 0.05:
            sleep(0.005)
        print('Arrive with resultant distance: {}'.format(self._dash.resDist))
        return 'Wait For Ball At TZ1'

    # ID 4, Wait For Ball At TZ1
    def __waitAtTZ1(self):
        print('Wait for 2 second')
        sleep(2)
        print('Done')
        return 'From TZ1 To TZA'

    # ID 5, From TZ1 To TZA
    def __TZ12TZA(self):
        self._dash.to(1.425, 3.055, 0)
        while self._dash.resDist >= 0.2:
            sleep(0.005)
        print('Arrive with resultant distance: {}'.format(self._dash.resDist))
        return 'From TZA To TZB'

    # ID 6, From TZA To TZB
    def __TZA2TZB(self):
        self._dash.to(1.425, 1.035, 0)
        while self._dash.resDist >= 0.2:
            sleep(0.005)
        print('Arrive with resultant distance: {}'.format(self._dash.resDist))
        return 'From TZB To TZ2'

    # ID 7, From TZB To TZ2
    def __TZB2TZ2(self):
        self._dash.to(3.775, 1.035, 0)
        while self._dash.resDist >= 0.05:
            sleep(0.005)
        print('Arrive with resultant distance: {}'.format(self._dash.resDist))
        return 'Wait For Ball At TZ2'

    # ID 8, Wait For Ball At TZ2
    def __waitAtTZ2(self):
        print('Wait for 2 second')
        sleep(2)
        print('Done')
        return 'From TZ2 To TZ3'

    # ID 9, From TZ2 To TZ3
    def __TZ22TZ3(self):
        self._dash.to(7.035, 1.035, 0)
        while self._dash.resDist >= 0.05:
            sleep(0.005)
        print('Arrive with resultant distance: {}'.format(self._dash.resDist))
        return 'Wait For Ball At TZ3'

    # ID 10, Wait For Ball At TZ3
    def __waitAtTZ3(self):
        print('Wait for 2 second')
        sleep(2)
        print('Done')
        return 'From TZ3 To ARSZ'

    # ID 11, From TZ3 To ARSZ
    def __TZ32ARSZ(self):
        self._dash.to(1.425, 1.035, 0)
        while self._dash.resDist >= 0.2:
            sleep(0.005)
        self._dash.to(0.55, 7.54, 0)
        while self._dash.resDist >= 0.05:
            sleep(0.005)
        self._dash.lock()
        print('Arrive with resultant distance: {}'.format(self._dash.resDist))
        return None
