from pyb import Pin, CAN
from time import sleep_ms
import struct

CAN(1).deinit()
can = CAN(1)
can.init(mode=CAN.NORMAL, prescaler=3, sjw=1, bs1=9, bs2=4)
# Baudrate is pyb.freq() / (sjw + bs1 + bs2) / prescaler = 1Mbps
can.setfilter(bank=0, mode=CAN.MASK16, fifo=0, params=(0x0, 0x0, 0x0, 0x0))

can.send(struct.pack('<ii', 3, 200000), 22)
