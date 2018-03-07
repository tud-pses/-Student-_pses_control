#ifndef PSES_SENSOR_NODE_HPP
#define PSES_SENSOR_NODE_HPP

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pses_control/sensorConfig.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>

#define DRIVEN_DISTANCE_PER_TICK 0.0251327412 // RAD_PER_TICK * WHEEL_RADIUS
#define WHEEL_RADIUS 6.5

class pses_sensor {

public:
    pses_sensor();       //Konstruktor der Klasse

    // Variables
    ros::NodeHandle nh;	// use this for global access

    //Functions
    void calculateVelocity();
    void reset();
    void set_powertrain();
    void publish_sensor_data();


private:
  // Variables
    std_msgs::UInt8 msg_hall_counter;
    std_msgs::Float64 msg_hall_dt, msg_hall_dt8, velocity, driven_distance, avg_speed;
    sensor_msgs::Range m_usr, m_usl, m_usf, m_usr_old, m_usl_old, m_usf_old;
    float tau;
    int data_count;
    std_msgs::Int16 target_speed, target_steering_angle;
    std_msgs::Int16 msg_motor_speed;
    dynamic_reconfigure::Server<pses_control::sensorConfig> server;
    geometry_msgs::TwistWithCovarianceStamped wheel_odom;


  // Subscriber
    //hall sensor
    ros::Subscriber sub_hall_counter;
    ros::Subscriber sub_hall_dt;
    ros::Subscriber sub_hall_dt8;
    //ultra sonic range sensor
    ros::Subscriber sub_usr;
    ros::Subscriber sub_usl;
    ros::Subscriber sub_usf;
    ros::Subscriber m_sub_motor_speed;

 //Advertiser
  //ros::Publisher pub_usr;
    ros::Publisher m_pub_motor_speed;
    ros::Publisher m_pub_steering;

    ros::Publisher m_pub_velocity;
    ros::Publisher m_pub_avg_speed;
    ros::Publisher m_pub_distance;
    ros::Publisher pub_usr;
    ros::Publisher m_pub_wheel_odom;
  // Functions
    //hall sensor
    static void hall_cnt_Callback(std_msgs::UInt8::ConstPtr hall_cnt_msg, std_msgs::UInt8* m_hall_cnt);
    static void hall_dt_Callback(std_msgs::Float64::ConstPtr hall_dt_msg, std_msgs::Float64* msg_hall_dt);
    static void hall_dt8_Callback(std_msgs::Float64::ConstPtr hall_dt8_msg, std_msgs::Float64* msg_hall_dt8);
    //ultra sonic range sensor
    static void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* m_usr, float* tau);
    static void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* m_usl);
    static void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* m_usf);
    static void motor_speed_Callback(std_msgs::Int16::ConstPtr motor_speed_msg, std_msgs::Int16* msg_motor_speed);

    static float lowpass(float input, float output_old, float tau);
    void param_callback(pses_control::sensorConfig &config, uint32_t level);




};

bool stop_request = false;
void signalHandler(int sig);

#endif //PSES_SENSOR_NODE_HPP
