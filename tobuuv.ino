#include "t200_controler.hpp"
#include <WString.h>

//T200 thruster_R(5);
T200 thruster_L(6);
T200 thruster_R(5);

String str;
int r_pulse;
int l_pulse;


void setup() {
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

    Serial.println("READY");
}


void loop() {

    if (Serial.available() > 3) {
        str = Serial.readString();
        str = str.substring(3);
        r_pulse = str.substring(0, 4).toInt();
        l_pulse = str.substring(4, 8).toInt();
    }

    thruster_R.speed(r_pulse);
    thruster_L.speed(l_pulse);
    thruster_R.run(5);
    thruster_L.run(5);
    
    //thruster_R.speed(1500);
    //thruster_R.run(5);
    //thruster_L.speed(1500);
    //thruster_L.run(5);
    
    //thruster_R.increase_astern();
    //thruster_L.speed(1800);
    //thruster_R.run(5);
    //thruster_L.run(5);
    /*
    String str = "";
    if (Serial.available() > 0) {
        str = Serial.readStringUntil('\n');
    }
    int comma = str.indexOf(',', 2);
    for (int )
    delay(100);
    */
}
