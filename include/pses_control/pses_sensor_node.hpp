#ifndef PSES_SENSOR_NODE_HPP
#define PSES_SENSOR_NODE_HPP

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>

class PsesSensor {

public:
    PsesSensor();       //Konstruktor der Klasse

    // Variables
    ros::NodeHandle nh;	// use this for global access

    //Functions
    void calculateVelocity();


private:
    // Variables
    std_msgs::UInt8 msg_hall_counter;
    std_msgs::Float64 msg_hall_dt, msg_hall_dt8;
    float velocity;
    // Subscriber
    ros::Subscriber sub_hall_counter;
    ros::Subscriber sub_hall_dt;
    ros::Subscriber sub_hall_dt8;

    // Functions
  static void hall_cnt_Callback(std_msgs::UInt8::ConstPtr hall_cnt_msg, std_msgs::UInt8* m_hall_cnt);
  static void hall_dt_Callback(std_msgs::Float64::ConstPtr hall_dt_msg, std_msgs::Float64* msg_hall_dt);
  static void hall_dt8_Callback(std_msgs::Float64::ConstPtr hall_dt8_msg, std_msgs::Float64* msg_hall_dt8);



};
#endif //PSES_SENSOR_NODE_HPP
