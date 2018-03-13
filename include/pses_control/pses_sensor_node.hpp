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

//describes the distance, which the car is moved within one tick of the wheel encoders
#define DRIVEN_DISTANCE_PER_TICK 0.0251327412 // RAD_PER_TICK * WHEEL_RADIUS

class pses_sensor {

public:
    /* NAME: pses_sensor
     * DESCRIPTION: constructor of the class pses_sensor
     *              initializes the subscribers and advertisers to ros topics
     *              set variables to default values and create parameter
     * INPUT: -
     * OUTPUT: -
     */
    pses_sensor();

    // Variables
    ros::NodeHandle nh; //node handle of the pses_sensor_node

    //Functions
    /* NAME: calculateVelocity
     * DESCRIPTION: calculates the actual velocity using the wheel Encoder, if the motor speed is unequal zero
     *              calculates the driven distance of the car, since the last start and the average speed over the last 5 seconds
     *              publishes the average_speed every 5 seconds
     * INPUT: -
     * OUTPUT: -
     */
    void calculateVelocity();

    /* NAME: publishSensorData
     * DESCRIPTION: calls the publish function of the advertisers for usr, wheel_odom and driven distance
     * INPUT: -
     * OUTPUT: -
     */
    void publishSensorData();


private:
  // Variables
    std_msgs::UInt8 msg_hall_counter; //number of ticks of the hall sensor
    std_msgs::Float64 msg_hall_dt, msg_hall_dt8; //time between one and eight ticks of the hall sensor
    std_msgs::Float64 velocity, driven_distance, avg_speed; //calculated velocity, driven distance and average speed of the car
    sensor_msgs::Range m_usr; // new value of the ultra sonic sensor right
    float usr_tau; //cutoff frequency for ultra sonic sensor right lowpass filter
    int data_count; //counter to calculate the average speed over time
    std_msgs::Int16 msg_motor_speed; //contains control value for motor speed
    dynamic_reconfigure::Server<pses_control::sensorConfig> server; //dynamic parameter server
    geometry_msgs::TwistWithCovarianceStamped wheel_odom; //contains velocity of the car


  // Subscriber
    ros::Subscriber sub_hall_counter; //subscriber for hall sensor counter
    ros::Subscriber sub_hall_dt;      //subscriber for hall sensor, time between one tick
    ros::Subscriber sub_hall_dt8;     //subscriber for hall sensor, time between eight ticks
    ros::Subscriber sub_usr;          //subscriber for ultra sonic sensor right
    ros::Subscriber sub_motor_speed_; //subscriber for set_motor_level_msg

 //Advertiser
    ros::Publisher pub_avg_speed;     //publishes average speed every 5 seconds
    ros::Publisher pub_distance;      //publishes driven distance of the car since last start
    ros::Publisher pub_usr;           //publishes filtered data of ultra sonic sensor right
    ros::Publisher pub_wheel_odom;    //publishes velocity of the car in TwistWithCovariance messag
  // Functions

    //----------------hall sensor----------------

    /* NAME: hall_cnt_Callback
     * DESCRIPTION: copies hall_cnt_msg into workspace variable
     * INPUT: std_msgs::UInt8::ConstPtr hall_cnt_msg: constant Pointer on received message from subscriber
     *        std_msgs::UInt8* m_hall_cnt: workspace variable, where the message is copied to
     * OUTPUT: -
     */
    static void hall_cnt_Callback(std_msgs::UInt8::ConstPtr hall_cnt_msg, std_msgs::UInt8* m_hall_cnt);
    /* NAME: hall_dt_Callback
     * DESCRIPTION: copies hall_dt_msg into workspace variable
     * INPUT: std_msgs::Float64::ConstPtr hall_dt_msg: constant Pointer on received message from subscriber
     *        std_msgs::Float64* msg_hall_dt: workspace variable, where the message is copied to
     * OUTPUT: -
     */
    static void hall_dt_Callback(std_msgs::Float64::ConstPtr hall_dt_msg, std_msgs::Float64* msg_hall_dt);
    /* NAME: hall_dt8_Callback
     * DESCRIPTION: copies hall_dt8_msg into workspace variable
     * INPUT: std_msgs::Float64::ConstPtr hall_dt8_msg: constant Pointer on received message from subscriber
     *        std_msgs::Float64* msg_hall_dt8: workspace variable, where the message is copied in
     * OUTPUT: -
     */
    static void hall_dt8_Callback(std_msgs::Float64::ConstPtr hall_dt8_msg, std_msgs::Float64* msg_hall_dt8);

    //----------------ultra sonic range sensor----------------

    /* NAME: usrCallback
     * DESCRIPTION: filters all zeros from the incoming values, the forwards the values to a lowpass filter
     *              then copies the output of the filter into workspace variable
     * INPUT: sensor_msgs::Range::ConstPtr usrMsg: constant Pointer on received message from subscriber
     *        sensor_msgs::Range* m_usr: workspace variable, where the message is copied in
     *        float* tau: cutoff frequency for the lowpass filter
     * OUTPUT: -
     */
    static void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* m_usr, float* tau);
    /* NAME: motorSpeedCallback
     * DESCRIPTION:  copies motor_speed_msg into workspace variable
     * INPUT: std_msgs::Int16::ConstPtr motor_speed_msg: constant Pointer on received message from subscriber
     *        std_msgs::Int16* msg_motor_speed: workspace variable, where the message is copied in
     * OUTPUT: -
     */
    static void motorSpeedCallback(std_msgs::Int16::ConstPtr motor_speed_msg, std_msgs::Int16* msg_motor_speed);
    /* NAME: lowpass
     * DESCRIPTION: lowpass filter of first order
     * INPUT: float input: new value, which should be filtered
     *        float output_old: last value, which was calculated by the lowpass filter
     *        float tau: cutoff frequency of the lowpass filter
     * OUTPUT: float: filtered result of input value
     */
    static float lowpass(float input, float output_old, float tau);
    /* NAME: paramCallback
     * DESCRIPTION:
     * INPUT: -
     * OUTPUT: -
     */
    void paramCallback(pses_control::sensorConfig &config, uint32_t level);

};

bool stop_request = false;
void signalHandler(int sig);

#endif //PSES_SENSOR_NODE_HPP
