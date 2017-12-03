/*
    Product : Arduino Due Elmo Driver
     Author : Ruoyang Xu
       Date : 2017/10/27
     Status : 框架移植完毕, 但上电后似乎什么也没有发生, 思考人生后决定使用RS-232调试接口
*/

#include <Arduino.h>
#include "variant.h"
#include <due_can.h>
#include <due_canopen.h>

// #define Serial SerialUSB

#define TYPE_INTEGER      0
#define TYPE_FLOAT        1
#define MO_OFF            0
#define MO_ON             1
#define POS_REL           0
#define POS_ABS           1
#define UM_SCM            0x02
#define TEST_ELMO_ID      0x01

void myDelay(uint8_t n) {
    CanOpen0.sendPDOMessage(0x129, 1, (uint8_t*) n);
}

void myPDO(uint8_t *cmd, uint8_t index, uint8_t type, uint32_t data) {
    uint8_t msg[8];
    msg[0] = *cmd++;
    msg[1] = *cmd;
    msg[2] = index;
    msg[3] = type << 7;
    msg[4] = data & 0xFF;
    msg[5] = (data & 0xFF00) >> 8;
    msg[6] = (data & 0xFF0000) >> 16;
    msg[7] = (data & 0xFF000000) >> 24;
    CanOpen0.sendPDOMessage(0x300 + TEST_ELMO_ID, 8, msg);
    myDelay(100);
}

void myPDOStr(uint8_t *cmd) {
    uint8_t msg[4];
    msg[0] = *cmd++;
    msg[1] = *cmd;
    msg[2] = 0x00;
    msg[3] = 0x00;
    CanOpen0.sendPDOMessage(0x300 + TEST_ELMO_ID, 4, msg);
    myDelay(100);
}

void mySDO(uint16_t index, uint8_t subIndex, uint32_t data) {
    SDO_FRAME SDOFrame;
    SDOFrame.nodeID = TEST_ELMO_ID;
    SDOFrame.cmd = SDO_WRITE;
    SDOFrame.index = index;
    SDOFrame.subIndex = subIndex;
    SDOFrame.dataLength = 4;
    SDOFrame.data[0] = data & 0xFF;
    SDOFrame.data[1] = (data & 0xFF00) >> 8;
    SDOFrame.data[2] = (data & 0xFF0000) >> 16;
    SDOFrame.data[3] = (data & 0xFF000000) >> 24;
    CanOpen0.sendSDORequest(&SDOFrame);
    myDelay(100);
}

void CAN_init(void)
{
    CanOpen0.setMasterMode();
    CanOpen0.begin(1000000, 0x10);
    // CanOpen0.setStateChangeCallback(onChangedState);
    // CanOpen0.setPDOCallback(onPDOReceive);
    // CanOpen0.setSDOReqCallback(onSDORxRequest);
    // CanOpen0.setSDOReplyCallback(onSDORxResponse);
    CanOpen0.sendNodeReset(0); //reset all nodes
    mySDO(0x1A00, 0x00, 0);
    mySDO(0x1A01, 0x00, 0);
    CanOpen0.sendNodeStart(0); //make all connected nodes go active
}

void setup() {
    // put your setup code here, to run once:
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    CAN_init();
    Serial.println("INIT OK");
    myPDO((uint8_t *)"MO", 0, TYPE_INTEGER, MO_OFF);
    myPDO((uint8_t *)"PM", 0, TYPE_INTEGER, 0x01);
    myPDO((uint8_t *)"AC", 0, TYPE_INTEGER, 1000000);
    myPDO((uint8_t *)"DC", 0, TYPE_INTEGER, 1000000);
    myPDO((uint8_t *)"UM", 0, TYPE_INTEGER, UM_SCM);
    myPDO((uint8_t *)"MO", 0, TYPE_INTEGER, MO_ON);
    myPDO((uint8_t *)"JV", 0, TYPE_INTEGER, 10000);
    myPDOStr((uint8_t *)"BG");
    Serial.println("OK");
}

void loop() {
    // put your main code here, to run repeatedly:
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
}