#include "t200_controler.hpp"

T200 thruster(5, 6);

void setup() {
    Serial.begin(9600);
    thruster.setup();
}

void loop() {
    thruster.increase_ahead();
    thruster.run(5);
    thruster.state();
}