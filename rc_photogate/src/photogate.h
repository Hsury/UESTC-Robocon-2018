#ifndef PHOTOGATE_H
#define PHOTOGATE_H

#define ACTIVE 0
#define INACTIVE 1

void isrWrapper();

class Photogate {
  public:
    Photogate();
    void isr();
    void begin(unsigned int pin, unsigned long duration = 5E4);
    void loop();
    int stat();
    void stop();

  private:
    unsigned int pin;
    unsigned long duration;
    volatile int stateMachine;
    volatile unsigned long startTimestamp;
    volatile unsigned long endTimestamp;
};

#endif
