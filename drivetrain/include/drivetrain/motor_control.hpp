#define _USE_MATH_DEFINES

#include "wiringPi.h"
#include "softPwm.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>


class RRB3 {

    private:
        const int right_pwm_pin = 15;
        const int right_1_pin = 12;
        const int right_2_pin = 6;
        const int right_en_a = 8;
        const int right_en_b = 9;
        const int left_pwm_pin = 5;
        const int left_1_pin = 0;
        const int left_2_pin = 7;
        const int left_en_a = 28;
        const int left_en_b = 29;
        const int sw1_pin = 14;
        const int sw2_pin = 13;
        const int led1_pin = 10;
        const int led2_pin = 11;
        const int oc1_pin = 3;
        const int oc2_pin = 2;
        const int trigger_pin = 1;
        const int echo_pin = 4;

        class motor {
            private:
                // Motor constants
                const float motor_voltage = 12;
                const float battery_voltage = 12;
                const int motor_delay = 200;  // ms
                const int trans_per_rot = 2652;
                const float wheel_radius = 3.81;  // cm
                const float wheel_circum = M_PI * pow(wheel_radius, 2);
                const float pwm_scale = motor_voltage / battery_voltage;

                // Pin configuration
                int pin_pwm;
                int pin_1;
                int pin_2;
                int pin_en_a;
                int pin_en_b;

                // Motor state
                int en_state_a = -1;
                int en_state_b = -1;
                int counts = 0;
                bool direction = 0;
            public:
                motor () {}
                motor(int _pin_pwm, int _pin_1, int _pin_2, int _pin_en_a, int _pin_en_b)
                    : pin_pwm(_pin_pwm), pin_1(_pin_1), pin_2(_pin_2),
                      pin_en_a(_pin_en_a), pin_en_b(_pin_en_b) {}

                void initialize() {
                    softPwmCreate(pin_pwm, 20, 100);  // Targeting a frequency of 500 Hz
                    pinMode(pin_1, OUTPUT);
                    pinMode(pin_2, OUTPUT);
                    pinMode(pin_en_a, INPUT);
                    pinMode(pin_en_b, INPUT);
                }

                void reset_counts() {
                    counts = 0;
                }

                int read_counts() {
                    std::pair<int,int> cur_en_states = read_encoders();
                    int cur_en_state_a = std::get<0>(cur_en_states);
                    int cur_en_state_b = std::get<1>(cur_en_states);
                    if ((cur_en_state_a != en_state_a) ||
                            (cur_en_state_b != en_state_b)) {
                        counts++;
                    }
                    en_state_a = cur_en_state_a;
                    en_state_b = cur_en_state_b;
                    return counts;
                }

                std::pair<int,int> read_encoders() {
                    int cur_a = digitalRead(pin_en_a);
                    int cur_b = digitalRead(pin_en_b);
                    return std::make_pair(cur_a, cur_b);
                }

                void set_driver_pins(float pwm, int dir) {
                    softPwmWrite(pin_pwm, pwm * 100 * pwm_scale);
                    digitalWrite(pin_1, dir);
                    digitalWrite(pin_2, !dir);
                }

                void set_motor(float pwm, int dir) {
                    set_driver_pins(pwm, dir);
                    direction = dir;
                }

                void stop() {
                    set_motor(0, 0);
                }

                float get_distance() {
                    return float(read_counts()) * wheel_circum / trans_per_rot;
                }
        };
        motor right_motor = motor(right_pwm_pin, right_1_pin, right_2_pin, right_en_a, right_en_b);
        motor left_motor = motor(left_pwm_pin, left_1_pin, left_2_pin, left_en_a, left_en_b);

   public:
        RRB3() {
            wiringPiSetup();
            right_motor.initialize();
            left_motor.initialize();

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
            left_motor.set_motor(left_pwm, left_dir);
            right_motor.set_motor(right_pwm, right_dir);
        }

        void go_distance(float distance=0, float speed=1, int left_dir=0, int right_dir=0) {
            float dist_traveled = 0;
            set_motors(speed, left_dir, speed, right_dir);
            while (dist_traveled < distance) {
                float right_dist_traveled = right_motor.get_distance();
                float left_dist_traveled = left_motor.get_distance();
                dist_traveled = (right_dist_traveled + left_dist_traveled) / 2.0;
            }
            stop_motors();
            right_motor.reset_counts();
            left_motor.reset_counts();
        }

        void forward(float distance=0, float speed=1) {
            go_distance(distance, speed, 0, 0);
        }

        void reverse(float distance=0, float speed=1) {
            go_distance(distance, speed, 1, 1);
        }

        void turn_left(float distance=0, float speed=1) {
            go_distance(distance, speed, 1, 0);
        }

        void turn_right(float distance=0, float speed=1) {
            go_distance(distance, speed, 0, 1);
        }

        void stop_motors() {
            left_motor.stop();
            right_motor.stop();
        }

   };

