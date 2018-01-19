import serial

class Elmo():
    '''
    UESTC 2018 Robocon Team
    ELMO Driver Package
    '''
    SPEED_MODE = 2
    POSITION_MODE = 4

    def __init__(self, port='/dev/elmo0', baudrate=19200):
        self._port = port
        self._baudrate = baudrate
        try:
            self._serial = serial.Serial(self._port, self._baudrate, timeout=0.1, write_timeout=0)
        except:
            pass
        self.reset()
    
    def read(self, cmd):
        try:
            self._serial.reset_input_buffer()
            self.write(cmd)
            while not self._serial.read_until(';'.encode()).startswith(cmd.encode()):
                pass
            return int(self._serial.read_until(';'.encode())[:-1])
        except:
            pass
    
    def write(self, cmd, arg=None):
        data = '{}={};'.format(cmd, round(arg)) if arg is not None else '{};'.format(cmd)
        try:
            self._serial.write(data.encode())
        except:
            pass
    
    def test(self):
        from time import sleep
        print('Testing Elmo...')
        print('  Port: {}'.format(self._port))
        print('  Baudrate: {}'.format(self._baudrate))
        print('  Step 1/4: Get serial number: {}'.format(self.sn))
        print('  Step 2/4: Forward')
        self.speed = 5E4
        self.begin()
        sleep(2)
        print('  Step 3/4: Break')
        self.stop()
        sleep(1)
        print('  Step 4/4: Reverse')
        self.speed = -5E4
        self.begin()
        sleep(2)
        self.stop()
        print('Done')
    
    @property
    def sn(self):
        return self.read('SN[4]')

    def reset(self):
        self.disable()
        self.accel = 1E8
        self.decel = 1E8
        self.mode = Elmo.SPEED_MODE
        self.enable()

    def disable(self):
        self.write('MO', 0)
    
    def enable(self):
        self.write('MO', 1)
    
    def begin(self):
        self.write('BG')
    
    def stop(self):
        self.write('ST')
    
    def setAccel(self, value=1E8):
        self.write('AC', value)
    
    def getAccel(self):
        return self.read('AC')

    accel = property(getAccel, setAccel)
    
    def setDecel(self, value=1E8):
        self.write('DC', value)
    
    def getDecel(self):
        return self.read('DC')

    decel = property(getDecel, setDecel)

    def setMode(self, value=SPEED_MODE):
        self.write('UM', value)
    
    def getMode(self):
        return self.read('UM')

    mode = property(getMode, setMode)

    def setSpeed(self, value=5E4):
        self.write('JV', value)
    
    def getSpeed(self):
        return self.read('JV')

    speed = property(getSpeed, setSpeed)

    def setPosition(self, value=0):
        self.write('PX', value)
    
    def getPosition(self):
        return self.read('PX')

    position = property(getPosition, setPosition)

if __name__=='__main__':
    elmo = Elmo('/dev/elmo0', 19200)
    elmo.test()
