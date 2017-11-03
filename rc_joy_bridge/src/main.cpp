#define X_PIN A0
#define Y_PIN A1
#define Z_PIN A2
#define ERR 64

#include <Arduino.h>
#include <ArduinoJson.h>

float x, y, z;

void encodeJson() {
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["x"] = x;
    json["y"] = y;
    json["z"] = z;
    json.printTo(Serial);
}

void getJoy() {
    x = analogRead(X_PIN) - 512;
    if (fabs(x) < ERR) x = 0;
    x = x / 512.0 * 3.0;
    y = - analogRead(Y_PIN) + 512;
    if (fabs(y) < ERR) y = 0;
    y = y / 512.0 * 3.0;
    z = - analogRead(Z_PIN) + 512;
    if (fabs(z) < ERR) z = 0;
    z = z / 512.0 * 360.0;
}

void setup() {
    Serial.begin(57600);
}

void loop() {
    getJoy();
    encodeJson();
    delay(100);
}
