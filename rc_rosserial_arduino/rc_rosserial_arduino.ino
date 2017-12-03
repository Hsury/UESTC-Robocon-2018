/*
   Product : Arduino ROS Serial Debugger
    Author : Ruoyang Xu
   Website : http://hsury.com/
*/

// roscore
// rosrun rosserial_python serial_node.py /dev/ttyUSB0
// rosrun turtlesim turtle_teleop_key

#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

SoftwareSerial SerialA(14, 12);

unsigned long timeStamp[1];

void sToggleFunc(const std_msgs::Empty& sToggleMsg) {
  Serial.println("---------- Subscribe ----------");
  Serial.println("Topic: /toggle");
  Serial.println("Type: std_msgs/Empty");
  digitalWrite(13, HIGH - digitalRead(13)); // blink the led
  Serial.print("LED: ");
  Serial.println(digitalRead(13));
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
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  Serial.println();
  Serial.println("Booting");
  SerialA.begin(57600);
  Serial.println("Software serial started");
  nh.initNode();
  nh.advertise(pVector);
  nh.subscribe(sToggle);
  nh.subscribe(sGo);
  Serial.println("System started");
}

void loop() {
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
      Serial.print("LED0: ");
      Serial.println(led0);
      Serial.print("LED1: ");
      Serial.println(led1);
      Serial.print("BUZZ: ");
      Serial.println(buzz);
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
  nh.spinOnce();
}

