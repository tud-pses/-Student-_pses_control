#include "pses_control/person_follow_control.hpp"


PersonFollowControl::PersonFollowControl() {
}

void PersonFollowControl::setGains(double kp_steering, double kd_steering, double kp_velocity, double kd_velocity) {
    kp_steering_ = kp_steering;
    kd_steering_ = kd_steering;
    kp_velocity_ = kp_velocity;
    kd_velocity_ = kd_velocity;

}

void PersonFollowControl::control() {
    double steering;
    double velocity;
}
