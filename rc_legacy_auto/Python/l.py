import serial
import threading
from time import sleep, time
from collections import deque
import struct
from math import sin, cos, pi, sqrt
import v

class ll():
    '''
    UESTC 2018 Robocon Team
    Low Layer Driver Package
    '''

    def __init__(self, port='COM13', baudrate=921600): #'/dev/stm32_vcp'
        self._port = port
        self._baudrate = baudrate
        try:
            self.startSerial()
            unpackThd = threading.Thread(target=self.__unpack)
            unpackThd.setDaemon(True)
            unpackThd.start()
            print("Listening thread started")
            '''
            displayThd = threading.Thread(target=self.__display)
            displayThd.setDaemon(True)
            displayThd.start()
            print("Display thread started")
            '''
        except:
            pass
    
    def startSerial(self):
        while (True):
            try:
                self._serial = serial.Serial(self._port, self._baudrate) #timeout=0.1
                print("{} open successfully".format(self._port))
                break
            except:
                print("{} open failed".format(self._port))
                sleep(0.25)
    
    def __unpack(self):
        buf = deque(maxlen=8)
        for i in range (8):
            buf.append(0)
        while (True):
            try:
                buf.append(self._serial.read())
            except:
                print("Serial communication error")
                self.startSerial()
            if (buf[0] == b'{' and buf[2] == b'|' and buf[7] == b'}'):
                try:
                    if (buf[1][0] >= 0xA1 and buf[1][0] <= 0xA9):
                        addr, data = struct.unpack('<xBxfx', b''.join(list(buf)))
                    else:
                        addr, data = struct.unpack('<xBxLx', b''.join(list(buf)))
                    if (addr == 0xFF):
                        print('OK')
                    elif (addr == 0x00):
                        v.aliveElmo1 = True if (data & (1 << 0)) else False
                        v.aliveElmo2 = True if (data & (1 << 1)) else False
                        v.aliveElmo3 = True if (data & (1 << 2)) else False
                        v.aliveGE = True if (data & (1 << 3)) else False
                        print('Base Elmo Head: {}'.format('Pass' if v.aliveElmo1 else 'Fail'))
                        print('Base Elmo Left: {}'.format('Pass' if v.aliveElmo2 else 'Fail'))
                        print('Base Elmo Right: {}'.format('Pass' if v.aliveElmo3 else 'Fail'))
                        print('Gyro Encoder: {}'.format('Pass' if v.aliveGE else 'Fail'))
                    elif (addr == 0xA1):
                        v.RealAccX = data
                    elif (addr == 0xA2):
                        v.RealAccY = data
                    elif (addr == 0xA3):
                        v.RealAccZ = data
                    elif (addr == 0xA4):
                        v.RealVelX = data
                    elif (addr == 0xA5):
                        v.RealVelY = data
                    elif (addr == 0xA6):
                        v.RealVelZ = data
                    elif (addr == 0xA7):
                        v.PosX = data
                    elif (addr == 0xA8):
                        v.PosY = data
                    elif (addr == 0xA9):
                        v.AngZ = data
                except:
                    print("Unpack error")
    
    def pack(self, addr, data):
        while (True):
            try:
                if (isinstance(data, int)):
                    self._serial.write(struct.pack('<BBBLB', ord('{'), addr, ord('|'), data, ord('}')))
                elif (isinstance(data, float)):
                    self._serial.write(struct.pack('<BBBfB', ord('{'), addr, ord('|'), data, ord('}')))
                else:
                    print("Wrong type")
                break
            except:
                print("Packet sent error")
                sleep(0.25)
        sleep(0.01)
    
    def __display(self):
        while (True):
            sleep(0.1)
            #print('X={}, Y={}, Z={}'.format(v.PosX, v.PosY, v.AngZ))
            print('X={}, Y={}, Z={}'.format(v.RealVelX, v.RealVelY, v.RealVelZ))
    
    def queryPeriph(self):
        self.pack(0x00, 0x00000000)
    
    def initBase(self):
        self.pack(0x00, 0x00000001)
        v.aliveElmo1 = False
        v.aliveElmo2 = False
        v.aliveElmo3 = False
        v.PosX = 0
        v.PosY = 0
        v.AngZ = 0
        v.VelX = 0
        v.VelY = 0
        v.VelZ = 0
    
    def initSling(self):
        self.pack(0x00, 0x00000002)
    
    def move(self):
        self.pack(0x00, 0x00000003)
    
    def lockpoint(self):
        self.pack(0x00, 0x00000004)
    
    def resetMCU(self):
        self.pack(0x00, 0xFFFFFFFF)
    
    def releaseBase(self):
        self.pack(0x10, 0x00000000)
        v.VelX = 0
        v.VelY = 0
        v.VelZ = 0
    
    def lockBase(self):
        self.pack(0x10, 0x00000001)
        v.VelX = 0
        v.VelY = 0
        v.VelZ = 0
    
    def applyVel(self):
        self.pack(0x10, 0x00000002)
    
    def presetVelX(self, X):
        self.pack(0x11, float(X))
        v.VelX = X
    
    def presetVelY(self, Y):
        self.pack(0x12, float(Y))
        v.VelY = Y
    
    def presetVelZ(self, Z):
        self.pack(0x13, float(Z))
        v.VelZ = Z
    
    def resetGE(self):
        self.pack(0x20, 0x00000000)
        v.PosX = 0
        v.PosY = 0
        v.AngZ = 0
    
    def clearGE(self):
        self.pack(0x20, 0x00000001)
        v.PosX = 0
        v.PosY = 0
        v.AngZ = 0
    
    def setPosX(self, X):
        self.pack(0x21, float(X))
        v.PosX = X
    
    def setPosY(self, Y):
        self.pack(0x22, float(Y))
        v.PosY = Y
    
    def setAngZ(self, Z):
        self.pack(0x23, float(Z))
        v.AngZ = Z
    
    def throwTZ1(self):
        self.pack(0x30, 0x00000001)
    
    def throwTZ2(self):
        self.pack(0x30, 0x00000002)
    
    def throwTZ3(self):
        self.pack(0x30, 0x00000003)
    
    '''
    def moveTest(self, X=0, Y=0, Duration=1):
        while (not (v.aliveElmo1 and v.aliveElmo2 and v.aliveElmo3 and v.aliveGE)):
            print('Not all periphrerals are online')
            self.initBase()
            self.initSling()
            sleep(3)
            self.queryPeriph()
            sleep(0.1)
        self.presetVelX(X)
        self.presetVelY(Y)
        self.applyVel()
        sleep(Duration)
        self.presetVelX(0)
        self.presetVelY(0)
        self.applyVel()
    
    def sche(self, deltaX=0, deltaY=0, Z=0):
        SAFE_DIST = 0.5
        MAX_SPEED = 0.5
        COEF_LINEAR = 5
        COEF_ANGULAR = 2.5
        startTS = time()
        startX = v.PosX
        startY = v.PosY
        startZ = v.AngZ
        goalX = startX + deltaX
        goalY = startY + deltaY
        goalZ = Z
        while (True):
            distX = goalX - v.PosX
            distY = goalY - v.PosY
            distZ = goalZ - v.AngZ
            if (distZ > 180):
                distZ = distZ - 360
            if (abs(distX) < 0.05 and abs(distY) < 0.05 and abs(distZ) < 5):
                break
            resDist = sqrt(distX ** 2 + distY ** 2)
            if resDist > SAFE_DIST:
                VelX = distX * MAX_SPEED / resDist
                VelY = distY * MAX_SPEED / resDist
            else:
                VelX = distX * COEF_LINEAR
                VelY = distY * COEF_LINEAR
            ResVel = sqrt(VelX ** 2 + VelY ** 2)
            VelZ = distZ * COEF_ANGULAR
            self.presetVelX(VelX)
            self.presetVelY(VelY)
            self.presetVelZ(VelZ)
            self.applyVel()
        self.lockBase()
        EndTS = time()
        print('Time passed: {}'.format(EndTS - startTS))
    '''
