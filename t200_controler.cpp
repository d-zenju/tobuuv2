#include "t200_controler.hpp"

T200::T200(int Thruster_Right_Pin, int Thruster_Left_Pin) {
    pin[0] = Thruster_Right_Pin;
    pin[1] = Thruster_Left_Pin;
}


T200::setup(void) {
    Thruster_R.attach(pin[0]);
    Thruster_L.attach(pin[1]);
    Thruster_R.writeMicroseconds(pulse[1]);
    Thruster_L.writeMicroseconds(pulse[1]);
}


T200::midship(void) {
    Thruster_R.writeMicroseconds(pulse[1]);
    Thruster_L.writeMicroseconds(pulse[1]);
}


T200::full_ahead(void) {
    Thruster_R.writeMicroseconds(pulse[2]);
    Thruster_L.writeMicroseconds(pulse[2]);
}


T200::full_astern(void) {
    Thruster_R.writeMicroseconds(pulse[0]);
    Thruster_L.writeMicroseconds(pulse[0]);
}


T200::increase_ahead(int accelerate = 1) {
    pulse_state[0] += accelerate;
    pulse_state[1] += accelerate;
    if (pulse_state[0] > pulse[2])
        pulse_state[0] = pulse[2];
    if (pulse_state[1] > pulse[2])
        pulse_state[1] = pulse[2];
}


T200::increase_right(int accelerate = 1) {
    pulse_state[0] += accelerate;
    if (pulse_state[0] > pulse[2])
        pulse_state[0] = pulse[2];
}


T200::increase_left(int accelerate = 1) {
    pulse_state[1] += accelerate;
    if (pulse_state[1] > pulse[2])
        pulse_state[1] = pulse[2];
}


T200::decrease_right(int accelerate = 1) {
    pulse_state[0] -= accelerate;
    if (pulse_state[0] < pulse[0])
        pulse_state[0] = pulse[0];
}


T200::decrease_left(int accelerate = 1) {
    pulse_state[1] -= accelerate;
    if (pulse_state[1] < pulse[0])
        pulse_state[1] = pulse[0];
}


T200::increase_astern(int accelerate = 1) {
    pulse_state[0] -= accelerate;
    pulse_state[1] -= accelerate;
    if (pulse_state[0] < pulse[0])
        pulse_state[0] = pulse[0];
    if (pulse_state[1] < pulse[0])
        pulse_state[1] = pulse[0];
}


T200::run(int fps) {
    frame_rate(fps);
    if (fps_flag == 1) {
        Thruster_R.writeMicroseconds(pulse_state[0]);
        Thruster_L.writeMicroseconds(pulse_state[1]);
    }
}


T200::state(void) {
    Serial.print(pulse_state[0]);
    Serial.print(" ");
    Serial.println(pulse_state[1]);
}

T200::frame_rate(int fps) {
    int subtime = millis() - oldtime;
    if (subtime > (1000 / fps)) {
        fps_flag = 1;
        oldtime = millis();
    } else {
        fps_flag = 0;
    }
}