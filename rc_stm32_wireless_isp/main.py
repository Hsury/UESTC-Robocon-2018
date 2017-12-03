from machine import Pin, UART
from uos import listdir
from utime import sleep_ms

class STM32_WIRELESS_ISP():
    '''STM32 Wireless ISP'''
    def __init__(self, boot0=14, reset=12, baud=460800):
        self.boot0 = Pin(boot0, Pin.OUT); self.boot0.off()
        self.reset = Pin(reset, Pin.OUT); self.reset.on()
        self.uart = UART(1, 460800, parity=2)
    
    def __send(self, data, ctrlByte=0):
        self.uart.write(bytearray([data]))
        if ctrlByte:
            self.uart.write(bytearray([data ^ 0xFF]))
    
    def isp(self):
        self.reset.off()
        self.boot0.on()
        self.reset.on()
        self.__send(0x7F, 0)
    
    def boot(self):
        self.reset.off()
        self.boot0.off()
        self.reset.on()
    
    def erase(self):
        self.__send(0x43, 1)
        self.__send(0xFF, 1)
    
    def hex2bin(self, hexFile='stm32.hex', binFile='stm32.bin'):
        if (hexFile in listdir()):
            print('Converting {} to {}...'.format(hexFile, binFile), end='')
            with open(hexFile, mode='rb') as h, open(binFile, mode='wb') as b:
                for line in h:
                    if (line[7: 9] == b'00'):
                        for i in range(int(line[1: 3], 16)):
                            b.write(bytes([int(line[9 + 2 * i: 11 + 2 * i], 16)]))
            print('Done')
        else:
            print('Cannot find {}'.format(hexFile))
    
    def flash(self, binFile='stm32.bin', addr=0x08000000, block=64):
        if (binFile in listdir()):
            print('Entering Bootloader...', end='')
            self.isp()
            sleep_ms(10)
            print('Done')
            print('Erasing Flash...', end='')
            self.erase()
            sleep_ms(100)
            print('Done')
            print('Uploading Firmware...', end='')
            with open(binFile, mode='rb') as b:
                while True:
                    binData = b.read(block)
                    binLength = len(binData)
                    if (binLength == 0):
                        break
                    self.__send(0x31, 1)
                    addrBytes = [(addr >> 24) & 0xFF, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, (addr >> 0) & 0xFF]
                    addrBytes.append(addrBytes[0] ^ addrBytes[1] ^ addrBytes[2] ^ addrBytes[3])
                    for i in addrBytes:
                        self.__send(i, 0)
                    crc = (block - 1) & 0xFF
                    self.__send(crc, 0)
                    for i in range(block):
                        binPart = binData[i] if (i < binLength) else 0xFF
                        crc ^= binPart
                        self.__send(binPart, 0)
                    self.__send(crc, 0)
                    addr += block
                    sleep_ms(10)
            print('Done')
            self.boot()
        else:
            print('Cannot find {}\nIf you uploaded a .hex file, use hex2bin() to create a .bin file first'.format(binFile))

if __name__ == '__main__':
    stm32 = STM32_WIRELESS_ISP()
