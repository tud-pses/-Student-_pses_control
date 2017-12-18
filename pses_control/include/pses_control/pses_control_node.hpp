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

    // Subscriber

    // Publisher

    // Functions
    void pidControl();  // set private in the future
    void driveTrajectory();
    void driveSteeringTest();
    void reset();

private:
    // Variables
    dynamic_reconfigure::Server<pses_control::controllerConfig> m_server;
    double m_kp, m_ki, m_kd;
    double m_e, m_e_sum, m_e_last;
    double m_target_value;
    sensor_msgs::Range m_usr, m_usf, m_usl;
    double m_ack_vel, m_ack_steering;
    ros::Time m_time_last = ros::Time::now();
    std_msgs::Int16 m_velocity, m_velocity_config, m_steering, m_steering_config, m_steering_min, m_steering_max;

    // Subscriber
    ros::Subscriber m_sub_usr;
    ros::Subscriber m_sub_usl;
    ros::Subscriber m_sub_usf;
    ros::Subscriber m_sub_ackermann_cmd;

    // Publisher
    ros::Publisher m_pub_velocity;
    ros::Publisher m_pub_steering;

    // Functions
    static void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* m_usl);
    static void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* m_usf);
    static void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* m_usr);
    void paramCallback(pses_control::controllerConfig &config, uint32_t level);
    static void ackermannCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& ackermannCmdMsg, double *m_ack_steering, double *m_ack_vel);
};

void signalHandler(int sig);
bool stop_request = false;


#endif //PSES_CONTROL_NODE_HPP
