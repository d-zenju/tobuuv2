#include "t200_controler.hpp"


T200::T200(int Thruster_Pin) {
    pin = Thruster_Pin;
}


void T200::setup(void) {
    Thruster.attach(pin);
    Thruster.writeMicroseconds(pulse[1]);
    delay(1000);
}


void T200::set_pulse(int mimimun, int midship, int maximum) {
    pulse[0] = mimimun;
    pulse[1] = midship;
    pulse[2] = maximum;
}


void T200::midship(void) {
    pulse_state = pulse[1];
}


void T200::full_ahead(void) {
    pulse_state = pulse[2];
}


void T200::full_astern(void) {
    pulse_state = pulse[0];
}


void T200::increase_ahead(int accelerate = 1) {
    pulse_state += accelerate;
    if (pulse_state > pulse[2])
        pulse_state = pulse[2];
}


void T200::increase_astern(int accelerate = 1) {
    pulse_state -= accelerate;
    if (pulse_state < pulse[0])
        pulse_state = pulse[0];
}


void T200::speed(int microseconds) {
    pulse_state = microseconds;
}


void T200::run(int fps) {
    frame_rate(fps);
    if (fps_flag == 1) {
        Thruster.writeMicroseconds(pulse_state);
    }
}


void T200::state(void) {
    Serial.println(pulse_state);
}


void T200::frame_rate(int fps) {
    int subtime = millis() - oldtime;
    if (subtime > (1000 / fps)) {
        fps_flag = 1;
        oldtime = millis();
    } else {
        fps_flag = 0;
    }
}