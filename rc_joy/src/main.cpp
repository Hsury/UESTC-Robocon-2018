#define X_PIN A0
#define Y_PIN A1
#define Z_PIN A2
#define ERR 64

#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>

float x, y, z;

ros::NodeHandle nh;
geometry_msgs::Vector3 vector3;
ros::Publisher pub("vel", &vector3);

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
    nh.initNode();
    nh.advertise(pub);
}

void loop() {
    getJoy();
    vector3.x = x;
    vector3.y = y;
    vector3.z = z * PI / 180;
    if (x or y or z) pub.publish(&vector3);
    nh.spinOnce();
}
