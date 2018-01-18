'''
description: 根据传家宝007_Elmo改写的MPY平台上的Elmo驱动
status: 未完成
issue: 报文发送失败，FIFO溢出
'''

from pyb import Pin, CAN
can = CAN(1)
can.init(mode=CAN.NORMAL, prescaler=3, sjw=1, bs1=9, bs2=4)
# Baudrate is pyb.freq() / (sjw + bs1 + bs2) / prescaler = 1Mbps
can.setfilter(bank=0, mode=CAN.MASK32, fifo=0, params=(0x0, 0x0))

COBID_NMT = 0x000
COBID_RSDO = 0x600
COBID_RPDO2 = 0x300

NMT_OPERATIONAL = 1
NMT_RESET_COMMUNICATION = 130

def NMT(elmoID, cmd):
    nmt = bytearray(2)
    nmt[0] = cmd
    nmt[1] = elmoID # Elmo ID
    can.send(nmt, id=COBID_NMT)

def RSDO(elmoID, index, subIndex, data):
    rsdo = bytearray(8)
    rsdo[0] = 0x22
    rsdo[1] = index & 0xFF
    rsdo[2] = (index & 0xFF00) >> 8
    rsdo[3] = subIndex
    rsdo[4] = data & 0xFF
    rsdo[5] = (data & 0xFF00) >> 8
    rsdo[6] = (data & 0xFF0000) >> 16
    rsdo[7] = (data & 0xFF000000) >> 24
    can.send(rsdo, id=COBID_RSDO + elmoID)

def RPDO2_DATA(elmoID, cmd, index, data):
    rpdo2 = bytearray(8)
    rpdo2[0] = cmd[0]
    rpdo2[1] = cmd[1]
    rpdo2[2] = index
    rpdo2[3] = 0 # INTEGER
    rpdo2[4] = data & 0xFF
    rpdo2[5] = (data & 0xFF00) >> 8
    rpdo2[6] = (data & 0xFF0000) >> 16
    rpdo2[7] = (data & 0xFF000000) >> 24
    can.send(rpdo2, id=COBID_RPDO2 + elmoID)

def RPDO2_STR(elmoID, cmd):
    rpdo2 = bytearray(4)
    rpdo2[0] = cmd[0]
    rpdo2[1] = cmd[1]
    rpdo2[2] = 0
    rpdo2[3] = 0
    can.send(rpdo2, id=COBID_RPDO2 + elmoID)

NMT(1, NMT_RESET_COMMUNICATION)
#RSDO(1, 0x1A00, 0, 0)
#RSDO(1, 0x1A01, 0, 0)
NMT(1, NMT_OPERATIONAL)
RPDO2_DATA(1, 'MO', 0, 1)
RPDO2_DATA(1, 'JV', 0, 10000)
RPDO2_STR(1, 'BG')
