#include <Arduino.h>
#include <photogate.h>

Photogate pg0;
Photogate pg1;

Photogate::Photogate() {
    this->pin = 0;;
    this->duration = 0;
    this->stateMachine = 0;
    this->startTimestamp = 0;
    this->endTimestamp = 0;
  }
  
  void isrWrapper() {
    pg0.isr();
    pg1.isr();
  }
  
  void Photogate::isr() {
    switch (this->stateMachine) {
      case 0:
        if (digitalRead(this->pin) == ACTIVE) this->stateMachine = 1;
        break;
      case 2:
        if (digitalRead(this->pin) == INACTIVE) this->stateMachine = 3;
        break;
    }
  }
  
  void Photogate::begin(unsigned int pin, unsigned long duration) {
    this->pin = pin;
    this->duration = duration;
    this->stateMachine = 0;
    pinMode(this->pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(this->pin), isrWrapper, CHANGE);
  }
  
  void Photogate::loop() {
    switch (this->stateMachine) {
        case 1:
        this->startTimestamp = micros();
        this->stateMachine = 2;
        break;
        case 2:
        this->endTimestamp = micros();
        if (this->endTimestamp - this->startTimestamp >= this->duration) this->stateMachine = 4;
        break;
        case 3:
        this->endTimestamp = micros();
        if (this->endTimestamp - this->startTimestamp < this->duration) this->stateMachine = 0;
        else this->stateMachine = 4;  // 防止发生意外卡住
        break;
        case 4:
        this->stateMachine = 0;
        break;
    }
  }
  
  int Photogate::stat() {
    return this->stateMachine;
  }

  void Photogate::stop() {
    detachInterrupt(digitalPinToInterrupt(this->pin));
  }

  /*
     状态机说明
     0: 等待光电门被挡住
     1: 中断触发, 获取光电门被挡住的开始时间戳
     2: 不断检测挡住时间是否达到过滤器阈值, 同时等待光电门被打开. 如果挡住时间达到了过滤器阈值, 直接判定球有效通过
     3: 中断触发, 获取光电门被打开的结束时间戳, 如果短于过滤器阈值, 回到0重新等待光电门被挡住
     4: 球有效球通过
  */
  