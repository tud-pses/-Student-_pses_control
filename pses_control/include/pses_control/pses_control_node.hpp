#ifndef PSES_CONTROL_NODE_HPP
#define PSES_CONTROL_NODE_HPP
#define PI 3.14159265

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Int16.h>
#include <stdlib.h>
#include <signal.h>
#include <dynamic_reconfigure/server.h>
#include <pses_control/controllerConfig.h>
#include <math.h>

class PsesControl {

public:
    PsesControl();

    // Variables
    ros::NodeHandle nh;	// use this for global access

    // Functions

    /*
     * NAME:        driveSteeringTest
     * DESCRIPTION: Test mode for measuring velocity and steering functions
     * INPUT:
     * OUTPUT:      void
    */
    void driveSteeringTest();

    /*
     * NAME:        pidControl
     * DESCRIPTION: PID controller for wall follow
     * INPUT:
     * OUTPUT:      void
    */
    void pidControl();

    /*
     * NAME:        reset
     * DESCRIPTION: Set steering and velocity to zero
     * INPUT:
     * OUTPUT:      void
    */
    void reset();

private:

    // Variables [note: command = Stellgroesse, value = physikalische Groesse]
    dynamic_reconfigure::Server<pses_control::controllerConfig> m_server;   // server for dynamic reconfigure
    double m_kp, m_ki, m_kd;                                                // respective gains Kp, Kd, Ki for PID controller
    double m_e, m_e_sum, m_e_last;                                          // respectively, error, error sum, error of previous iteration (used in PID controller)
    double m_target_value;                                                  // target value for distance to wall (used in PID controller)
    sensor_msgs::Range m_usr, m_usf, m_usl;                                 // ultrasound sensor values (right, front, left)
    double m_ack_vel, m_ack_steering;                                       // velocity and steering values
    ros::Time m_time_last = ros::Time::now();                               // time of previous iteration (used in PID controller)
    std_msgs::Int16 m_velocity, m_steering;                                 // velocity and steering commands
    std_msgs::Int16 m_velocity_config, m_steering_config, m_steering_min, m_steering_max; //config values for steering and velocity command, min and max for steering command (used in dynamic reconfigure)

    // Subscriber
    ros::Subscriber m_sub_usr;  // subscriber for right ultrasound sensor value
    ros::Subscriber m_sub_usl;  // subscriber for left ultrasound sensor value
    ros::Subscriber m_sub_usf;  // subscriber for front ultrasound sensor value

    // Publisher
    ros::Publisher m_pub_velocity;  // publisher for velocity command
    ros::Publisher m_pub_steering;  // publisher for steering command

    // Functions

    /*
     * NAME:        uslCallback
     * DESCRIPTION: Stores value of left ultrasound sensor in workspace
     * INPUT:       sensor_msgs::Range::ConstPtr uslMsg - left ultrasound sensor value
     *              sensor_msgs::Range* m_usl - left ultrasound sensor value (workspace)
     * OUTPUT:      void
    */
    static void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* m_usl);

    /*
     * NAME:        usfCallback
     * DESCRIPTION: Stores value of front ultrasound sensor in workspace
     * INPUT:       sensor_msgs::Range::ConstPtr usfMsg - front ultrasound sensor value
     *              sensor_msgs::Range* m_usf - front ultrasound sensor value (workspace)
     * OUTPUT:      void
    */
    static void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* m_usf);

    /*
     * NAME:        usrCallback
     * DESCRIPTION: Stores value of right ultrasound sensor in workspace
     * INPUT:       sensor_msgs::Range::ConstPtr usrMsg - right ultrasound sensor value (workspace)
     *              sensor_msgs::Range* m_usr - right ultrasound sensor value (workspace)
     * OUTPUT:      void
    */
    static void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* m_usr);

    /*
     * NAME:        paramCallback
     * DESCRIPTION: Callback for dynamic reconfigure
     * INPUT:       pses_control::controllerConfig &config
     *              uint32_t level
     * OUTPUT:      void
    */
    void paramCallback(pses_control::controllerConfig &config, uint32_t level);
};

/*
 * NAME:        signalHandler
 * DESCRIPTION: Sets stop request to true when program is aborted
 * INPUT:       int sig - aborting signal
 * OUTPUT:      void
*/
void signalHandler(int sig);
bool stop_request = false;


#endif //PSES_CONTROL_NODE_HPP
