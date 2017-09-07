#include "t200_controler.hpp"
#include "mpu9250.hpp"
#include <String.h>


// THRUSTER
T200 thruster_L(6);
T200 thruster_R(5);
int r_pulse;
int l_pulse;


// MPU9250
MPU9250 imu;


String str;
int start;

double accel[3] = {0.0, 0.0, 0.0};
double gyro[3] = {0.0, 0.0, 0.0};
double magnet[3] = {0.0, 0.0, 0.0};
double temp = 0.0;


void setup() {/******* ADDRESS ********/
    Serial.begin(38400);
    while (!Serial) {
        ;   // wait for serial port to connect.  Needed for native USB port only
    }
    Serial.setTimeout(10);

    // setup thruster
    thruster_R.set_pulse(1180, 1500, 1820);
    thruster_L.set_pulse(1180, 1500, 1820);
    thruster_R.setup();
    thruster_L.setup();

    // setup MPU9250
    imu.init();

    start = millis();

    Serial.println("READY");
}


void loop() {

    if (Serial.available() > 0) {
        str = Serial.readString();
        r_pulse = str.substring(0, 4).toInt();
        l_pulse = str.substring(4, 8).toInt();
    } else {    // DON'T receive serial signal -> Midship
        int sub_time = millis() - start;
        if (sub_time > 5000) {
            r_pulse = 1500;
            l_pulse = 1500;
        }
    }

    // thruster control
    thruster_R.speed(r_pulse);
    thruster_L.speed(l_pulse);
    thruster_R.run(5);
    thruster_L.run(5);

    // print sensor
    imu.getSensor();
    imu.calcSensor();
    imu.calcYPR();
    printSensor();
}


void printSensor() {
    // Accel
    for (int i = 0; i < 3; i++) {
        Serial.print(imu.accel[i]);
        Serial.print(',');
    }
    // Gyro
    for (int i = 0; i < 3; i++) {
        Serial.print(imu.gyro[i]);
        Serial.print(',');
    }
    // Magnet
    for (int i = 0; i < 3; i++) {
        Serial.print(imu.magnet[i]);
        Serial.print(',');
    }
    // Yaw, Pitch, Roll
    for (int i = 0; i < 3; i++) {
        Serial.print(imu.ypr[i]);
        Serial.print(',');
    }
    // Temp
    Serial.println(imu.temp);
}