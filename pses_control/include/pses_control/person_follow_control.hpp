#ifndef PERSON_FOLLOW_CONTROL_HPP
#define PERSON_FOLLOW_CONTROL_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

class PersonFollowControl {

    public:
        PersonFollowControl();

        /*
         * NAME:        setGains
         * DESCRIPTION: Sets parameter of controllers
         * INPUT:       double& kp_steering
                        double& kd_steering
                        double& kp_velocity
                        double& kd_velocity
         * OUTPUT:      void
        */
        void setGains(double& kp_steering, double& kd_steering, double& kp_velocity, double& kd_velocity);

        /*
         * NAME:        control
         * DESCRIPTION:	Controls velocity and steering of system
         * INPUT:       geometry_msgs::PoseStamped& target
                        float& steering
                        float& velocity
         * OUTPUT:      void
        */
        void control(geometry_msgs::PoseStamped& target, float& steering, float& velocity);


    private:
        double kp_steering_, kd_steering_, kp_velocity_, kd_velocity_, last_error_x_, last_error_z_;
		// time stamp of last iteration
        ros::Time last_time_;
		// indicates if it is the first run of the object
        bool first_run_ = true;

};

#endif // PERSON_FOLLOW_CONTROL_HPP
