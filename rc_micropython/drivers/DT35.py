from pyb import CAN
from time import sleep_ms
import struct

k=0

def cb0(bus, reason):
  '''
  print('cb0')
  if reason == 0:
      print('pending')
  if reason == 1:
      print('full')
  if reason == 2:
      print('overflow')
  '''
  global k
  k=1

CAN(1).deinit()
can = CAN(1)
can.init(mode=CAN.NORMAL, extframe=False, prescaler=3, sjw=1, bs1=9, bs2=4)
# Baudrate is pyb.freq() / (sjw + bs1 + bs2) / prescaler = 1Mbps
can.setfilter(bank=0, mode=CAN.MASK16, fifo=0, params=(0, 0, 0, 0))
can.rxcallback(0, cb0)
while True:
    if k==1:
        print(struct.unpack('<L', can.recv(0)[3])[0])
        k=0
