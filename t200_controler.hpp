#ifndef _T200_CONROLER_hpp
#define _T200_CONROLER_hpp

#include <Arduino.h>
#include <Servo.h>

class T200 {
private:
    // servo
    Servo Thruster;

    // set thruster pulse(micro sec) 0: full astern, 1: midship, 2: full ahead
    int pulse[3] = {1100, 1500, 1900};

    // set thruster pin
    int pin = -1;

    // thruster pulse state
    int pulse_state = pulse[1];

    // FPS
    int oldtime = millis();
    int fps_flag = 0;
    void frame_rate(int fps);

public:
    T200(int Thruster_Pin);
    void setup(void);
    void set_pulse(int mimimun, int midship, int maximum);
    void midship(void);
    void full_ahead(void);
    void full_astern(void);
    void increase_ahead(int accelerate = 1);
    void increase_astern(int accelerate = 1);
    void speed(int microseconds);
    void run(int fps);
    void state(void);
};

#endif