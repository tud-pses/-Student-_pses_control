#ifndef PERSON_FOLLOW_CONTROL_HPP
#define PERSON_FOLLOW_CONTROL_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

class PersonFollowControl {

    public:
        PersonFollowControl();
        void setGains(double& kp_steering, double& kd_steering, double& kp_velocity, double& kd_velocity);
        void control(geometry_msgs::PoseStamped& target, float& steering, float& velocity);


    private:
        double kp_steering_, kd_steering_, kp_velocity_, kd_velocity_, last_error_x_, last_error_z_;
        ros::Time last_time_;
        bool first_run_ = true;

};

#endif // PERSON_FOLLOW_CONTROL_HPP
