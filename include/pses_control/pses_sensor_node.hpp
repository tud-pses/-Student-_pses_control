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
    std_msgs::Float64 msg_hall_dt, msg_hall_dt8;
    // Subscriber
    ros::Subscriber sub_hall_counter;
    ros::Subscriber sub_hall_dt;
    ros::Subscriber sub_hall_dt8;

    // Functions
    void hall_cntCallback(std_msgs::UInt8 hall_cnt_msg, std_msgs::UInt8* m_hall_cnt);
    void calculateVelocity();


};
#endif //PSES_SENSOR_NODE_HPP
