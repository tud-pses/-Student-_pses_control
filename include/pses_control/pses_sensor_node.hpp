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

class pses_sensor {

public:
    pses_sensor();       //Konstruktor der Klasse

    // Variables
    ros::NodeHandle nh;	// use this for global access
    ros::Publisher pub_usr;
    sensor_msgs::Range m_usr, m_usl, m_usf, m_usr_old, m_usl_old, m_usf_old;

    //Functions
    void calculateVelocity();


private:
  // Variables
    std_msgs::UInt8 msg_hall_counter;
    std_msgs::Float64 msg_hall_dt, msg_hall_dt8;
 //sensor_msgs::Range m_usr, m_usl, m_usf, m_usr_old, m_usl_old, m_usf_old;
    float velocity, tau;

  // Subscriber
    //hall sensor
    ros::Subscriber sub_hall_counter;
    ros::Subscriber sub_hall_dt;
    ros::Subscriber sub_hall_dt8;
    //ultra sonic range sensor
    ros::Subscriber sub_usr;
    ros::Subscriber sub_usl;
    ros::Subscriber sub_usf;

  //Advertiser
 //   ros::Publisher pub_usr;
  // Functions
    //hall sensor
    static void hall_cnt_Callback(std_msgs::UInt8::ConstPtr hall_cnt_msg, std_msgs::UInt8* m_hall_cnt);
    static void hall_dt_Callback(std_msgs::Float64::ConstPtr hall_dt_msg, std_msgs::Float64* msg_hall_dt);
    static void hall_dt8_Callback(std_msgs::Float64::ConstPtr hall_dt8_msg, std_msgs::Float64* msg_hall_dt8);
    //ultra sonic range sensor
    static void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* m_usr);
    static void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* m_usl);
    static void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* m_usf);

    static float lowpass(float input, float output_old, float tau);




};
#endif //PSES_SENSOR_NODE_HPP
