#include "t200_controler.hpp"

//T200 thruster_R(5);
T200 thruster_L(6);
T200 thruster_R(5);


void setup() {
    Serial.begin(9600);
    while (!Serial) {
        ;   // wait for serial port to connect.  Needed for native USB port only
    }
    Serial.setTimeout(10);

    thruster_R.setup();
    thruster_L.setup();

    Serial.println("READY");
}


void loop() {

    thruster_R.speed(1500);
    thruster_R.run(5);
    thruster_L.speed(1500);
    thruster_L.run(5);
    
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
