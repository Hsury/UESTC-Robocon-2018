/*
   Product : ESP8266 ROS Serial Debugger
    Author : Ruoyang Xu
   Website : http://hsury.com/
*/

// roslaunch rosserial_server socket.launch
// rosrun turtlesim turtle_teleop_key

#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

SoftwareSerial SerialA(14, 12);

const char* ssid = "Xiaomi_33C7";
const char* password = "duoguanriben8";
IPAddress server(192, 168, 31, 250);  // Set the rosserial socket server IP address
const uint16_t serverPort = 11411;  // Set the rosserial socket server port
unsigned long timeStamp[1];

void sToggleFunc(const std_msgs::Empty& sToggleMsg) {
  Serial.println("---------- Subscribe ----------");
  Serial.println("Topic: /toggle");
  Serial.println("Type: std_msgs/Empty");
  digitalWrite(4, HIGH - digitalRead(4)); // blink the led
  Serial.printf("LED: %d", digitalRead(4));
  Serial.println();
  Serial.println();
}

void sGoFunc(const geometry_msgs::Twist& sGoMsg) {
  Serial.println("---------- Subscribe ----------");
  Serial.println("Topic: /turtle1/cmd_vel");
  Serial.println("Type: geometry_msgs/Twist");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& root_linear = root.createNestedObject("linear");
  JsonObject& root_angular = root.createNestedObject("angular");
  root_linear["x"] = sGoMsg.linear.x;
  root_linear["y"] = sGoMsg.linear.y;
  root_linear["z"] = sGoMsg.linear.z;
  root_angular["x"] = sGoMsg.angular.x;
  root_angular["y"] = sGoMsg.angular.y;
  root_angular["z"] = sGoMsg.angular.z;
  root.printTo(SerialA);
  SerialA.println();
  Serial.print("Linear x: ");
  Serial.println(sGoMsg.linear.x);
  Serial.print("Linear y: ");
  Serial.println(sGoMsg.linear.y);
  Serial.print("Linear z: ");
  Serial.println(sGoMsg.linear.z);
  Serial.print("Angular x: ");
  Serial.println(sGoMsg.angular.x);
  Serial.print("Angular y: ");
  Serial.println(sGoMsg.angular.y);
  Serial.print("Angular z: ");
  Serial.println(sGoMsg.angular.z);
  Serial.println();
}

ros::NodeHandle nh;
geometry_msgs::Vector3 pVectorMsg;
ros::Publisher pVector("stm32", &pVectorMsg);
ros::Subscriber<std_msgs::Empty> sToggle("toggle", &sToggleFunc);
ros::Subscriber<geometry_msgs::Twist> sGo("turtle1/cmd_vel", &sGoFunc);

void setup() {
  pinMode(4, OUTPUT);
  Serial.begin(115200);
  Serial.println();
  Serial.println("Booting");
  SerialA.begin(57600);
  Serial.println("Software serial started");
  Serial.printf("SSID: %s", ssid);
  Serial.println();
  Serial.printf("Password: %s", password);
  Serial.println();
  Serial.print("Connecting to WiFi ");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  for (int i = 0; i < 20; i++) {
    delay(500);
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print(">");
    } else {
      break;
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" FAIL");
    Serial.println("Restarting");
    ESP.restart();
  }
  Serial.println(" OK");
  Serial.print("Local IP address: ");
  Serial.println(WiFi.localIP());
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("Robocon_2018");
  // ArduinoOTA.setPassword((const char *)"duoguanriben8");
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA started");
  Serial.print("ROS IP address: ");
  Serial.println(server);
  Serial.print("ROS Port: ");
  Serial.println(serverPort);
  Serial.print("Connecting to ROS ");
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertise(pVector);
  nh.subscribe(sToggle);
  nh.subscribe(sGo);
  for (int i = 0; i < 5; i++) {
    delay(500);
    if (!nh.connected()) {
      Serial.print(">");
    } else {
      break;
    }
    nh.spinOnce();
  }
  if (!nh.connected()) {
    Serial.println(" FAIL");
    Serial.println("Restarting");
    ESP.restart();
  }
  Serial.println(" OK");
  Serial.println("System started");
}

void loop() {
  static int attempt;
  ArduinoOTA.handle();
  if (SerialA.available()) {
    Serial.println("----------- Publish -----------");
    Serial.println("Topic: /stm32");
    Serial.println("Type: geometry_msgs/Vector3");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& root = jsonBuffer.parse(SerialA);
    if (root.success()) {
      /*
        const char* jsonString = root["string"];
        if (jsonString != NULL) {
        pStringMsg.data = jsonString;
        pString.publish(&pStringMsg);
        }
      */
      int led0 = root["led0"];
      int led1 = root["led1"];
      int buzz = root["buzz"];
      pVectorMsg.x = led0;
      pVectorMsg.y = led1;
      pVectorMsg.z = buzz;
      pVector.publish(&pVectorMsg);
      Serial.printf("LED0: %d", led0);
      Serial.println();
      Serial.printf("LED1: %d", led1);
      Serial.println();
      Serial.printf("BUZZ: %d", buzz);
      Serial.println();
    } else {
      Serial.println("JSON parsing failed");
    }
    Serial.println();
  }
  /*
    if (timeStamp[0] == 0 or millis() - timeStamp[0] > 1000) {

    timeStamp[0] = millis();
    }
  */
  if (nh.connected()) {
    if (attempt != 0) {
      Serial.println(" OK");
      attempt = 0;
    }
  } else {
    if (attempt == 0) {
      Serial.print("Lost connection ");
    } else if (attempt == 5) {
      Serial.println(" FAIL");
      Serial.println("Restarting");
      ESP.restart();
    } else {
      Serial.print(">");
    }
    delay(500);
    attempt++;
  }
  nh.spinOnce();
}
