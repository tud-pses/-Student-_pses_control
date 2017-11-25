#ifndef PSES_SENSOR_NODE_HPP
#define PSES_SENSOR_NODE_HPP

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <stdlib.h>
#include <math.h>

class PsesSensor {

public:
    PsesSensor();       //Konstruktor der Klasse

    // Variables
    ros::NodeHandle nh;	// use this for global access

private:
    // Variables
    std_msgs::UInt8 msg_hall_counter;

    // Subscriber
    ros::Subscriber subscriber_hall_counter;

    // Functions
    //static void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* m_usl);

};
#endif //PSES_SENSOR_NODE_HPP
