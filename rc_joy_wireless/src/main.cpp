#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>

float x, y, z, gyro;
unsigned long timeStamp;

const char* ssid = "ROS";
const char* password = "robocon2018";
IPAddress server(192, 168, 1, 201);
const uint16_t serverPort = 11411;

void decodeJson();
void gyroCB(const geometry_msgs::Vector3& data);

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R2, 4, 5, U8X8_PIN_NONE);
ros::NodeHandle nh;
geometry_msgs::Vector3 vector3;
ros::Publisher pub("vel", &vector3);
ros::Subscriber<geometry_msgs::Vector3> sub("gyro", &gyroCB);

void setup() {
    Serial.begin(57600);
    u8g2.begin();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.clearBuffer();
    u8g2.setCursor(0, 10);
    u8g2.print("UESTC Robocon 2018");
    u8g2.setCursor(0, 20);
    u8g2.print("SSID: ");
    u8g2.print(ssid);
    u8g2.setCursor(0, 30);
    u8g2.print("Passwd: ");
    u8g2.print(password);
    u8g2.setCursor(0, 40);
    u8g2.print("Server: ");
    u8g2.print(server);
    u8g2.setCursor(0, 50);
    u8g2.print("Port: ");
    u8g2.print(serverPort);
    u8g2.setCursor(0, 60);
    u8g2.print("Connecting... ");
    u8g2.sendBuffer();
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) delay(50);
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.advertise(pub);
    nh.subscribe(sub);
    u8g2.print("OK");
    u8g2.sendBuffer();
    delay(1000);
}

void loop() {
    if ((timeStamp == 0) or (millis() - timeStamp > 250)) {
        u8g2.clearBuffer();
        u8g2.setCursor(0, 10);
        if (nh.connected()) u8g2.print("ROS Online");
        else u8g2.print("ROS Offline");
        u8g2.setCursor(0, 20);
        u8g2.print("X Spd: ");
        u8g2.print(x);
        u8g2.print(" m/s");
        u8g2.setCursor(0, 30);
        u8g2.print("Y Spd: ");
        u8g2.print(y);
        u8g2.print(" m/s");
        u8g2.setCursor(0, 40);
        u8g2.print("Z Spd: ");
        u8g2.print(z);
        u8g2.print(" deg/s");
        u8g2.setCursor(0, 50);
        u8g2.print("Gyro: ");
        u8g2.print(gyro);
        u8g2.print(" deg");
        u8g2.sendBuffer();
        timeStamp = millis();
    }
    decodeJson();
    vector3.x = x;
    vector3.y = y;
    vector3.z = z * PI / 180;;
    if (nh.connected()) pub.publish(&vector3);
    nh.spinOnce();
}

void decodeJson() {
    if (Serial.available()) {
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parse(Serial);
        if (json.success()) {
            x = json["x"];
            y = json["y"];
            z = json["z"];
        } else {
            x = 0;
            y = 0;
            z = 0;
        }
    }
}

void gyroCB(const geometry_msgs::Vector3& data) {
    gyro = data.z / PI * 180;
}