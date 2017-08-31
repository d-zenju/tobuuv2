#ifndef _T200_CONROLER_hpp
#define _T200_CONROLER_hpp

#include "Arduino.h"
#include "Servo.h"

class T200 {
private:
    // servo(Right, Left)
    Servo Thruster_R;
    Servo Thruster_L;

    // set thruster pulse(micro sec) 0: full astern, 1: midship, 2: full ahead
    int pulse[3] = {1100, 1500, 1900};

    // set thruster pin number 0: Right, 1: Left
    int pin[2] = {-1, -1};

    // thruster pulse state 0: right, 1: left
    int pulse_state[2] = {pulse[1], pulse[1]};

    // FPS
    int oldtime = millis();
    int fps_flag = 0;
    frame_rate(int fps);

public:
    T200(int Thruster_Right_Pin, int Thruster_Left_Pin);
    setup(void);
    midship(void);
    full_ahead(void);
    full_astern(void);
    increase_ahead(int accelerate = 1);
    increase_astern(int accelerate = 1);
    increase_right(int accelerate = 1);
    increase_left(int accelerate = 1);
    decrease_right(int accelerate = 1);
    decrease_left(int accelerate = 1);
    run(int fps);
    state(void);
};

#endif