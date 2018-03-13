// Controls velocity and steering with PD controllers

#include "pses_control/person_follow_control.hpp"

PersonFollowControl::PersonFollowControl() {
    kp_steering_ = 0.0;
    kd_steering_ = 0.0;
    kp_velocity_ = 0.0;
    kd_velocity_ = 0.0;
    last_error_x_ = 0.0;
    last_error_z_ = 0.0;
}

// Sets parameter of controllers
void PersonFollowControl::setGains(double& kp_steering, double& kd_steering, double& kp_velocity, double& kd_velocity) {
    kp_steering_ = kp_steering;
    kd_steering_ = kd_steering;
    kp_velocity_ = kp_velocity;
    kd_velocity_ = kd_velocity;

}
// Controls velocity and steering of system with a PD controller
void PersonFollowControl::control(geometry_msgs::PoseStamped& target, float& steering, float& velocity) {
    if(first_run_) {
        last_time_ = ros::Time::now();
        first_run_ = false;
    }
    double dt = ros::Time::now().toSec() - last_time_.toSec();
    steering = kp_steering_ * target.pose.position.x + kd_steering_ * (target.pose.position.x - last_error_x_ )/ dt;
    velocity = kp_velocity_ * target.pose.position.z + kd_velocity_ * (target.pose.position.z - last_error_z_) / dt;
	// Limit manipulated value according to limits of system
    if (velocity > 500.0) velocity = 500.0;
    else if (velocity < -500.0) velocity = -500.0;
    if (steering > 1000.0) steering = 1000.0;
    else if (steering < -1000.0) steering = -1000.0;
    last_time_ = ros::Time::now();
    last_error_x_ = target.pose.position.x;
}
