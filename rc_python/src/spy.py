from math import sin, cos, pi, sqrt
import os
import threading
import time

class Spy():
    '''UESTC 2018 Robocon Team
    Spy Package
    '''
    isSpying = False

    def __init__(self, dash):
        self._dash = dash
        self._dataDir = os.path.dirname(os.getcwd()) + os.sep + 'data'
    
    def __watch(self):
        try:
            filename = 'rc_trace_{}.txt'.format(time.strftime("%Y%d%m_%H%M%S"))
            path = self._dataDir + os.sep
            if os.path.exists(path + 'rc_trace.txt'):
                os.remove(path + 'rc_trace.txt')
            os.symlink(path + filename, path + 'rc_trace.txt')
            with open(path + filename, 'w') as fobj:
                while Spy.isSpying:
                    fobj.writelines('{} {} {} '.format(self._dash._merge.data[0], self._dash._merge.data[1], self._dash._merge.data[2]))
                    fobj.writelines('{} {} {} '.format(self._dash._goal[0], self._dash._goal[1], self._dash._goal[2]))
                    fobj.writelines('{} {} {}\n'.format(self._dash._speed[0], self._dash._speed[1], self._dash._speed[2]))
                    time.sleep(0.01)
        except:
            pass

    def begin(self):
        if not Spy.isSpying:
            thd = threading.Thread(target=self.__watch)
            thd.setDaemon(True)
            thd.start()
            Spy.isSpying = True
    
    def stop(self):
        Spy.isSpying = False
