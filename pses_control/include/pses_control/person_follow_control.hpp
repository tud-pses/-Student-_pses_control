#ifndef PERSON_FOLLOW_CONTROL_HPP
#define PERSON_FOLLOW_CONTROL_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class PersonFollowControl {

    public:
        PersonFollowControl();
        void setGains(double kp_steering, double kd_steering, double kp_velocity, double kd_velocity);
        void control();


    private:
        double kp_steering_, kd_steering_, kp_velocity_, kd_velocity_;
        double kp_;

};

#endif // PERSON_FOLLOW_CONTROL_HPP
