#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <ArduinoJson.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C screen0(U8G2_R0, U8X8_PIN_NONE, SCL, SDA);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C screen1(U8G2_R0, U8X8_PIN_NONE, SCL, SDA);
WiFiClient client;

#define rc2018_map_width 64
#define rc2018_map_height 64

static unsigned char rc2018_map[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x21, 0x00, 0x00, 0x02, 
    0x40, 0x00, 0x08, 0x80, 0x21, 0x00, 0x00, 0x02, 0x40, 0x00, 0x08, 0x80, 
    0x21, 0x00, 0x00, 0x02, 0x40, 0x00, 0x08, 0x80, 0x21, 0x00, 0x00, 0x02, 
    0x40, 0x00, 0x08, 0x80, 0x21, 0x00, 0x00, 0x02, 0x40, 0x00, 0x08, 0x80, 
    0x21, 0x00, 0x00, 0x02, 0x40, 0x00, 0x08, 0x80, 0x21, 0x00, 0x00, 0x02, 
    0x40, 0x00, 0x08, 0x80, 0x21, 0x00, 0x00, 0x02, 0x40, 0x00, 0x08, 0x80, 
    0x3F, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0x0F, 0x80, 0x01, 0x00, 0x00, 0x02, 
    0x40, 0x00, 0x08, 0x80, 0x01, 0x00, 0x00, 0x02, 0x40, 0x00, 0x08, 0x80, 
    0x01, 0x00, 0x00, 0x02, 0x40, 0x00, 0x08, 0x80, 0x01, 0x00, 0x00, 0x82, 
    0x41, 0x00, 0x08, 0x80, 0x01, 0x00, 0x00, 0xC2, 0x43, 0x00, 0x08, 0x80, 
    0x01, 0x00, 0x00, 0x62, 0x46, 0x00, 0x08, 0x80, 0x01, 0x00, 0x00, 0x62, 
    0x46, 0x00, 0x08, 0x80, 0x3F, 0x00, 0x00, 0xC2, 0x43, 0x00, 0x08, 0x80, 
    0x21, 0x00, 0x00, 0x82, 0xC1, 0xFF, 0x0F, 0x80, 0x21, 0x00, 0x00, 0x02, 
    0x00, 0x00, 0x08, 0x80, 0x21, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x80, 
    0x21, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x80, 0x3F, 0x00, 0x00, 0x02, 
    0x00, 0x00, 0x08, 0x80, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x80, 
    0x01, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x80, 0x01, 0x00, 0x00, 0x02, 
    0x00, 0x00, 0x08, 0x80, 0x3F, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x80, 
    0x21, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x80, 0x21, 0x00, 0x00, 0x02, 
    0x00, 0x00, 0x08, 0x80, 0x21, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x80, 
    0x21, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x80, 0x3F, 0x00, 0x1E, 0xC2, 
    0xC3, 0xFF, 0x0F, 0x80, 0x01, 0xF0, 0xFF, 0xC3, 0x43, 0x78, 0x00, 0xFC, 
    0x01, 0x10, 0x00, 0x00, 0x40, 0x00, 0x00, 0x84, 0x01, 0x10, 0x00, 0x00, 
    0x40, 0x00, 0x00, 0x84, 0x01, 0x10, 0x00, 0x00, 0x40, 0x00, 0x00, 0x84, 
    0x01, 0x10, 0x00, 0x00, 0x40, 0x00, 0x00, 0x84, 0x01, 0x10, 0x00, 0x00, 
    0x40, 0x00, 0x00, 0xFC, 0x01, 0x10, 0x00, 0x00, 0x40, 0x00, 0x00, 0x80, 
    0x01, 0x10, 0x00, 0x00, 0x40, 0x00, 0x00, 0x80, 0x01, 0x10, 0x00, 0x00, 
    0x40, 0x00, 0x00, 0x80, 0x01, 0x10, 0x00, 0x00, 0x40, 0x00, 0x00, 0xFC, 
    0x01, 0x10, 0x00, 0x00, 0x40, 0x00, 0x00, 0x84, 0x01, 0x10, 0x00, 0x00, 
    0x40, 0x00, 0x00, 0x84, 0x01, 0x10, 0x00, 0x00, 0x40, 0x00, 0x00, 0x84, 
    0x01, 0xF0, 0xFF, 0x83, 0x41, 0x00, 0x00, 0x84, 0x01, 0x10, 0x00, 0xC2, 
    0x43, 0x00, 0x00, 0xFC, 0x01, 0x10, 0x00, 0x62, 0x46, 0x00, 0x00, 0x80, 
    0x01, 0x10, 0x00, 0x62, 0x46, 0x00, 0x00, 0x80, 0x01, 0x10, 0x00, 0xC2, 
    0x43, 0x00, 0x00, 0x80, 0x01, 0x10, 0x00, 0x82, 0x41, 0x00, 0x00, 0x80, 
    0x01, 0x10, 0x00, 0x02, 0x40, 0x00, 0x00, 0x80, 0x01, 0x10, 0x00, 0x02, 
    0x40, 0x00, 0x00, 0x80, 0x01, 0x10, 0x00, 0x02, 0x40, 0x00, 0x00, 0x80, 
    0x01, 0xF0, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0xFC, 0x01, 0x10, 0x00, 0x02, 
    0x40, 0x00, 0x00, 0x84, 0x01, 0x10, 0x00, 0x02, 0x40, 0x00, 0x00, 0x84, 
    0x01, 0x10, 0x00, 0x02, 0x40, 0x00, 0x00, 0x84, 0x01, 0x10, 0x00, 0x02, 
    0x40, 0x00, 0x00, 0x84, 0x01, 0x10, 0x00, 0x02, 0x40, 0x00, 0x00, 0x84, 
    0x01, 0x10, 0x00, 0x02, 0x40, 0x00, 0x00, 0x84, 0x01, 0x10, 0x00, 0x02, 
    0x40, 0x00, 0x00, 0x84, 0x01, 0x10, 0x00, 0x02, 0x40, 0x00, 0x00, 0x84, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

const char* ssid     = "Robocon";
const char* password = "duoguanriben8";
const char* host     = "192.168.1.233";
const int tcpPort    = 2018;
boolean tcpReady = true;
int page = -1;
int subPage = -1;
unsigned long timeStamp[4];
int fps = 0;
float dash_dist[3];
float dash_goal[3];
boolean dash_locked;
float dash_position[3];
float dash_speed[3];
float feed_status[4];

void screenInit(int addr0, int addr1);
void display(int page);
void wifiConnect();
void remoteConnect();
void parseJson();

void setup() {
    Serial.begin(115200);
    screenInit(0x78, 0x7A);
    wifiConnect();
    remoteConnect();
    display(2);
}

void loop() {
    timeStamp[0] = millis();
    if (millis() - timeStamp[3] > 500) {
        if (WiFi.status() != WL_CONNECTED) {
            wifiConnect();
        }
        if (!client.connected()) {
            remoteConnect();
        }
        timeStamp[3] = millis();
    }
    display(2);
    parseJson();
    fps = 1000 / (millis() - timeStamp[0]);
}

void screenInit(int addr0, int addr1) {
    screen0.setI2CAddress(addr0);
    screen0.begin();
    screen0.enableUTF8Print();
    screen1.setI2CAddress(addr1);
    screen1.begin();
    screen1.enableUTF8Print();
}

void display(int page) {
    switch (page) {
        case 0:
            screen0.clearBuffer();
            screen0.setFont(u8g2_font_unifont_t_chinese3);
            screen0.setCursor(24, 30);
            screen0.print("正在连接到");
            screen0.setCursor(52, 50);
            screen0.print("WiFi");
            screen0.sendBuffer();
            screen1.clearBuffer();
            screen1.setFont(u8g2_font_unifont_t_chinese3);
            screen1.setCursor(0, 15);
            screen1.print("SSID");
            screen1.setCursor(0, 49);
            screen1.print("Password");
            screen1.setFont(u8g2_font_profont12_tf);
            screen1.setCursor(0, 27);
            screen1.print(ssid);
            screen1.setCursor(0, 61);
            screen1.print(password);
            screen1.sendBuffer();
            break;
        case 1:
            screen0.clearBuffer();
            screen0.setFont(u8g2_font_unifont_t_chinese3);
            screen0.setCursor(24, 30);
            screen0.print("正在连接到");
            screen0.setCursor(48, 50);
            screen0.print("主机");
            screen0.sendBuffer();
            screen1.clearBuffer();
            screen1.setFont(u8g2_font_unifont_t_chinese3);
            screen1.setCursor(0, 15);
            screen1.print("Address");
            screen1.setCursor(0, 49);
            screen1.print("Port");
            screen1.setFont(u8g2_font_profont12_tf);
            screen1.setCursor(0, 27);
            screen1.print(host);
            screen1.setCursor(0, 61);
            screen1.print(tcpPort);
            screen1.sendBuffer();
            break;
        case 2:
            screen0.clearBuffer();
            screen0.drawXBM(0, 0, rc2018_map_width, rc2018_map_height, rc2018_map);
            screen0.setFont(u8g2_font_unifont_t_chinese3);
            screen0.setCursor(66, 15);
            screen0.print("FPS");
            screen0.setCursor(66, 49);
            screen0.print("广播");
            screen0.setFont(u8g2_font_profont12_tf);
            screen0.setCursor(66, 27);
            screen0.print(fps);
            screen0.setCursor(66, 61);
            screen0.print(int(feed_status[0]));
            screen0.setDrawColor(2);
            int goal_center[2] = {1 + round(dash_goal[0] / 14 * (rc2018_map_width - 3)), 62 - round(dash_goal[1] / 14 * (rc2018_map_height - 3))};
            screen0.drawPixel(goal_center[0], goal_center[1]);
            screen0.drawPixel(goal_center[0] - 1, goal_center[1] - 1);
            screen0.drawPixel(goal_center[0] + 1, goal_center[1] - 1);
            screen0.drawPixel(goal_center[0] + 1, goal_center[1] + 1);
            screen0.drawPixel(goal_center[0] - 1, goal_center[1] + 1);
            screen0.setDrawColor(1);
            if (millis() - timeStamp[1] > 150) {
                int position_center[2] = {1 + round(dash_position[0] / 14 * (rc2018_map_width - 3)), 62 - round(dash_position[1] / 14 * (rc2018_map_height - 3))};
                screen0.drawBox(position_center[0] - 1, position_center[1] - 1, 3, 3);
                if (millis() - timeStamp[1] > 300) {
                    timeStamp[1] = millis();
                }
            }
            screen0.sendBuffer();
            switch (subPage) {
                case 0:
                    screen1.clearBuffer();
                    screen1.setFont(u8g2_font_unifont_t_chinese3);
                    screen1.setCursor(0, 15);
                    screen1.print("当前坐标");
                    screen1.setCursor(0, 49);
                    screen1.print("目的坐标");
                    screen1.setFont(u8g2_font_profont12_tf);
                    screen1.setCursor(0, 27);
                    screen1.print(dash_position[0], 3);
                    screen1.print("m ");
                    screen1.print(dash_position[1], 3);
                    screen1.print("m ");
                    screen1.print(dash_position[2] / PI * 180, 1);
                    screen1.print("°");
                    screen1.setCursor(0, 61);
                    screen1.print(dash_goal[0], 3);
                    screen1.print("m ");
                    screen1.print(dash_goal[1], 3);
                    screen1.print("m ");
                    screen1.print(dash_goal[2] / PI * 180, 1);
                    screen1.print("°");
                    screen1.sendBuffer();
                    break;
                case 1:
                    screen1.clearBuffer();
                    screen1.setFont(u8g2_font_unifont_t_chinese3);
                    screen1.setCursor(0, 15);
                    screen1.print("偏移");
                    screen1.setCursor(0, 49);
                    screen1.print("速度");
                    screen1.setFont(u8g2_font_profont12_tf);
                    screen1.setCursor(0, 27);
                    screen1.print(sqrt(pow(dash_dist[0], 2) + pow(dash_dist[1], 2)), 3);
                    screen1.print("m ");
                    screen1.print(dash_dist[2] / PI * 180, 1);
                    screen1.print("°");
                    screen1.setCursor(0, 61);
                    screen1.print(sqrt(pow(dash_speed[0], 2) + pow(dash_speed[1], 2)), 3);
                    screen1.print("m/s ");
                    screen1.print(dash_speed[2] / PI * 180, 1);
                    screen1.print("°/s");
                    screen1.sendBuffer();
                    break;
            }
            if (millis() - timeStamp[2] > 3000) {
                subPage++;
                if (subPage > 1) {
                    subPage = 0;
                }
                timeStamp[2] = millis();
            }
            break;
    }
}

void wifiConnect() {
    display(0);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        display(0);
        delay(50);
    }
}

void remoteConnect() {
    display(1);
    client.connect(host, tcpPort);
    while (!client.connected()) {
        display(1);
        client.connect(host, tcpPort);
        delay(50);
    }
    tcpReady = true;
}

void parseJson() {
    DynamicJsonBuffer jsonBuffer;
    if (tcpReady) {
        JsonArray& json = jsonBuffer.createArray();
        //json.add("print(self._dash.goal)");
        char buffer[512];
        json.printTo(buffer, sizeof(buffer));
        client.print(buffer);
        tcpReady = false;
    }
    if (client.available()) {
        JsonObject& json = jsonBuffer.parse(client);
        if (json.success()) {
            dash_dist[0] = json["dash_dist"][0];
            dash_dist[1] = json["dash_dist"][1];
            dash_dist[2] = json["dash_dist"][2];
            dash_goal[0] = json["dash_goal"][0];
            dash_goal[1] = json["dash_goal"][1];
            dash_goal[2] = json["dash_goal"][2];
            dash_locked = json["dash_locked"];
            dash_position[0] = json["dash_position"][0];
            dash_position[1] = json["dash_position"][1];
            dash_position[2] = json["dash_position"][2];
            dash_speed[0] = json["dash_speed"][0];
            dash_speed[1] = json["dash_speed"][1];
            dash_speed[2] = json["dash_speed"][2];
            feed_status[0] = json["feed_status"][0];
            feed_status[1] = json["feed_status"][1][0];
            feed_status[2] = json["feed_status"][1][1];
            feed_status[3] = json["feed_status"][1][2];
            tcpReady = true;
        }
    }
}
