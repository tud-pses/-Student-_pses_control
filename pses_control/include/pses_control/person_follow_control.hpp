#ifndef PERSON_FOLLOW_CONTROL_HPP
#define PERSON_FOLLOW_CONTROL_HPP

#include <ros/ros.h>

using namespace cv;
using namespace std;

class PersonFollowControl {

    public:
        PersonFollowControl();
        void PersonFollowControl::setGains(double kp_steering, double kd_steering, double kp_velocity, double kd_velocity);
        void PersonFollowControl::control();


    private:
        double kd_;
        double kp_;

};

#endif // PERSON_FOLLOW_CONTROL_HPP
