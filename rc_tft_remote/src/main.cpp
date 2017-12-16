#include <WiFi.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Fonts/msyh7pt7b.h>
#include <Fonts/msyh10pt7b.h>
#include <Fonts/msyh20pt7b.h>
#include "bitmap/uestc.h"
#include "bitmap/robocon.h"
#include "bitmap/frame.h"

#define TFT_CS -1
#define TFT_DC 16
#define TFT_RST 5

#define ROBOT 1
#define WIRELESS 1
#define BAUDRATE 115200

const char* ssid = "Mecanum";
const char* password = "duoguanriben8";
const char* host = "192.168.2.100";
const int tcpPort = 2018;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
WiFiClient client;

boolean tcpReady = true;
unsigned long fpsTS, refreshTS, flashTS, watchdogTS;
int fps;
float dist[3];
float goal[3];
boolean locked;
float position[3];
float speed[3];
boolean flow_busy;
int flow_id;
String flow_task;
int pixelToDraw[2];

void alignCenterPrint(String text, int16_t x, int16_t y) {
    int16_t x0, y0;
    x0 = tft.getCursorX();
    y0 = tft.getCursorY();
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds((char *)text.c_str(), 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(x - w / 2, y - h / 2);
    tft.print(text);
    tft.setCursor(x0, y0);
}

void conn() {
    tft.fillScreen(ILI9341_WHITE);
    tft.drawRGBBitmap((320 - ROBOCON_WIDTH) / 2, (240 - ROBOCON_HEIGHT) / 2, robocon, ROBOCON_WIDTH, ROBOCON_HEIGHT); // Display Robocon 2018 LOGO
    tft.setCursor(0, 0);
    tft.print(F("Robot: "));
    if (ROBOT == 0) tft.println(F("Manual"));
    else if (ROBOT == 1) tft.println(F("Automatical"));
    tft.print(F("Mode: "));
    if (WIRELESS == 0) {
        tft.println(F("Wire"));
        tft.print(F("Baudrate: "));
        tft.println(BAUDRATE);
        tft.setFont(&msyh10pt7b);
        alignCenterPrint("Waiting for ACK", 160, 210);
        tft.setFont();
    } else if (WIRELESS == 1) {
        tft.println(F("Wireless"));
        tft.print(F("SSID: "));
        tft.println(ssid);
        tft.print(F("Password: "));
        tft.println(password);
        tft.setFont(&msyh10pt7b);
        alignCenterPrint("Connecting to Wi-Fi", 160, 210);
        tft.setFont();
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) delay(50);
        tft.print(F("Client IP: "));
        tft.println(WiFi.localIP());
        tft.print(F("Server IP: "));
        tft.println(host);
        tft.print(F("Server Port: "));
        tft.println(tcpPort);
        tft.fillRect(67, 185, 188, 21, ILI9341_WHITE);
        tft.setFont(&msyh10pt7b);
        alignCenterPrint("Connecting to TCP Server", 160, 210);
        tft.setFont();
        client.connect(host, tcpPort);
        while (!client.connected()) {
            client.connect(host, tcpPort);
            delay(50);
        }
        tcpReady = true;
    }
}

void setup() {
    Serial.begin(BAUDRATE);
    tft.begin();
    tft.setRotation(1);
    tft.setTextColor(ILI9341_BLACK);
    tft.setTextWrap(false);
    tft.fillScreen(ILI9341_WHITE);
    tft.drawRGBBitmap((320 - UESTC_WIDTH) / 2, (240 - UESTC_HEIGHT) / 2, uestc, UESTC_WIDTH, UESTC_HEIGHT); // Display UESTC LOGO
    delay(1000);
    conn();
    // Draw frame
    tft.fillScreen(ILI9341_WHITE);
    tft.drawRGBBitmap(0, 0, frame, FRAME_WIDTH, FRAME_HEIGHT);
}

void loop() {
    fpsTS = micros();
    // Check line status
    if (millis() - watchdogTS > 500 and (WiFi.status() != WL_CONNECTED or !client.connected())) {
        conn();
        tft.fillScreen(ILI9341_WHITE);
        tft.drawRGBBitmap(0, 0, frame, FRAME_WIDTH, FRAME_HEIGHT);
        watchdogTS = millis();
    }
    // Parse JSON data
    DynamicJsonBuffer jsonBuffer;
    if (tcpReady) {
        JsonArray& json = jsonBuffer.createArray();
        //json.add("print('Hello, world!')");
        char buffer[512];
        json.printTo(buffer, sizeof(buffer));
        client.print(buffer);
        tcpReady = false;
    }
    if (client.available()) {
        JsonObject& json = jsonBuffer.parse(client);
        if (json.success()) {
            dist[0] = json["dist"][0];
            dist[1] = json["dist"][1];
            dist[2] = json["dist"][2];
            goal[0] = json["goal"][0];
            goal[1] = json["goal"][1];
            goal[2] = json["goal"][2];
            locked = json["locked"];
            position[0] = json["position"][0];
            position[1] = json["position"][1];
            position[2] = json["position"][2];
            speed[0] = json["speed"][0];
            speed[1] = json["speed"][1];
            speed[2] = json["speed"][2];
            flow_busy = json["flow_busy"];
            flow_id = json["flow_id"];
            const char* flow_task_orig = json["flow_task"];
            flow_task = String(flow_task_orig);
            tcpReady = true;
        }
    }
    // Time to refresh TFT
    if (millis() - refreshTS > 100) {
        // Print lock status
        tft.setFont(&msyh10pt7b);
        tft.setTextColor(ILI9341_WHITE);
        if (locked == true) {
            tft.fillRect(141, 82, 179, 58, ILI9341_RED);
            alignCenterPrint("BASE LOCKED", 229, 125);
        } else {
            tft.fillRect(141, 82, 179, 58, ILI9341_BLUE);
            alignCenterPrint("BASE UNLOCKED", 229, 125);
        }
        // Print speed
        tft.fillRect(180, 1, 140, 81, ILI9341_WHITE);
        tft.setFont(&msyh7pt7b);
        tft.setTextColor(ILI9341_BLACK);
        tft.setCursor(180, 14);
        tft.print(speed[0], 3);
        tft.setCursor(229, 14);
        tft.print(speed[1], 3);
        tft.setCursor(278, 14);
        tft.print(speed[2] * 180 / PI, 2);
        // Print position
        tft.setCursor(180, 35);
        tft.print(position[0], 3);
        tft.setCursor(229, 35);
        tft.print(position[1], 3);
        tft.setCursor(278, 35);
        tft.print(position[2] * 180 / PI, 2);
        // Print goal
        tft.setCursor(180, 56);
        tft.print(goal[0], 3);
        tft.setCursor(229, 56);
        tft.print(goal[1], 3);
        tft.setCursor(278, 56);
        tft.print(goal[2] * 180 / PI, 2);
        // Print distance
        tft.setCursor(180, 77);
        tft.print(dist[0], 3);
        tft.setCursor(229, 77);
        tft.print(dist[1], 3);
        tft.setCursor(278, 77);
        tft.print(dist[2] * 180 / PI, 2);
        // Print Task
        tft.fillRect(0, 163, 230, 28, ILI9341_WHITE);
        tft.fillRect(0, 212, 230, 28, ILI9341_WHITE);
        if (flow_busy) {
            alignCenterPrint(flow_task, 115, 184);
            alignCenterPrint("<Waiting for command>", 115, 234);
        } else {
            alignCenterPrint("<Waiting for command>", 115, 184);
            alignCenterPrint(flow_task, 115, 234);
        }
        // Print status code
        tft.fillRect(231, 161, 89, 64, ILI9341_WHITE);
        tft.setFont(&msyh20pt7b);
        alignCenterPrint(String(flow_id), 274, 225);
        tft.setFont();
        // Print FPS
        tft.fillRect(232, 232, 60, 8, ILI9341_WHITE);
        tft.setCursor(232, 232);
        tft.print("FPS ");
        tft.print(fps);
        // Flash the robot
        constrain(pixelToDraw[0], 0, 139);
        constrain(pixelToDraw[1], 0, 139);
        if (millis() - flashTS > 250) {
            tft.writePixel(pixelToDraw[0], pixelToDraw[1], ILI9341_PURPLE);
            flashTS = millis();
        } else if (millis() - flashTS > 0) {
            tft.writePixel(pixelToDraw[0], pixelToDraw[1], ILI9341_ORANGE);
        }
        if (pixelToDraw[0] != round(position[0] * 10) or (140 - pixelToDraw[1]) != round(position[1] * 10)) {
            tft.writePixel(pixelToDraw[0], pixelToDraw[1], ILI9341_PURPLE);
            //if (pixelToDraw[0] != -1 and pixelToDraw[1] != -1) tft.drawLine(pixelToDraw[0], pixelToDraw[1], round(position[0] * 10), 140 - round(position[1] * 10), ILI9341_PURPLE);
            pixelToDraw[0] = round(position[0] * 10);
            pixelToDraw[1] = 140 - round(position[1] * 10);
        }
        refreshTS = millis();
    }
    delay(10); // Make FPS around 100
    fps = 1E6 / (micros() - fpsTS);
}
