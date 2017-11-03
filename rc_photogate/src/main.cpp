#include <Arduino.h>
#include <photogate.h>

#define CLIP_PIN 23
#define DURATION 2000

extern Photogate pg0;
extern Photogate pg1;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CLIP_PIN, OUTPUT);
    pg0.begin(22, 5E5);
    pg1.begin(24, 5E5);
}

void loop() {
    pg0.loop();
    pg1.loop();
    if (pg0.stat() == 4 or pg1.stat() == 4) {
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(CLIP_PIN, LOW);
        delay(DURATION);
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(CLIP_PIN, HIGH);
    }
}
