#include "wiringPi.h"
#include "softPwm.h"
#include <chrono>
#include <thread>
#include <iostream>


class RRB3 {

    private:
        const int motor_delay = 200;  // ms
        const float right_pwm_pin = 15;
        const float right_1_pin = 12;
        const float right_2_pin = 6;
        const float left_pwm_pin = 5;
        const float left_1_pin = 0;
        const float left_2_pin = 7;
        const float sw1_pin = 14;
        const float sw2_pin = 13;
        const float led1_pin = 10;
        const float led2_pin = 11;
        const float oc1_pin = 3;
        const float oc2_pin = 2;
        const float trigger_pin = 1;
        const float echo_pin = 4;

        float left_pwm = 0;  // Ranges 0 -> 1
        float right_pwm = 0;  // Ranges 0 -> 1
        float pwm_scale = 0;

        float old_left_dir = -1;
        float old_right_dir = -1;

        float motor_voltage, battery_voltage;
    public:
        RRB3(float motor_v, float battery_v) {
            wiringPiSetup();
            pwm_scale = motor_v / battery_v;
            if (pwm_scale > 1) {
                std::cout << "WARNING: Motor voltage is higher than battery votage.";
                std::cout << "Motor may run slow." << std::endl;
            }
            motor_voltage = motor_v;
            battery_voltage = battery_v;

            softPwmCreate(left_pwm_pin, 20, 100);  // Targeting a frequency of 500 Hz
            pinMode(left_1_pin, OUTPUT);
            pinMode(left_2_pin, OUTPUT);

            softPwmCreate(right_pwm_pin, 20, 100);
            pinMode(right_1_pin, OUTPUT);
            pinMode(right_2_pin, OUTPUT);

            pinMode(led1_pin, OUTPUT);
            pinMode(led2_pin, OUTPUT);

            pinMode(oc1_pin, OUTPUT);
            pinMode(oc2_pin, OUTPUT);

            pinMode(sw1_pin, INPUT);
            pinMode(sw2_pin, INPUT);
            pinMode(trigger_pin, OUTPUT);
            pinMode(echo_pin, INPUT);
        }

        void set_motors(float left_pwm, int left_dir, float right_pwm, int right_dir) {
            if (old_left_dir != left_dir || old_right_dir != right_dir) {
                set_driver_pins(0, 0, 0, 0);    // Stop motors between sudden changes of direction
                std::this_thread::sleep_for(std::chrono::milliseconds(motor_delay));
            }
            set_driver_pins(left_pwm, left_dir, right_pwm, right_dir);
            old_left_dir = left_dir;
            old_right_dir = right_dir;
        }

        void set_driver_pins(float left_pwm, int left_dir, float right_pwm, int right_dir) {
            softPwmWrite(left_pwm_pin, left_pwm * 100 * pwm_scale);
            digitalWrite(left_1_pin, left_dir);
            digitalWrite(left_2_pin, !left_dir);
            softPwmWrite(right_pwm_pin, right_pwm * 100 * pwm_scale);
            digitalWrite(right_1_pin, right_dir);
            digitalWrite(right_2_pin, !right_dir);
        }
    };

