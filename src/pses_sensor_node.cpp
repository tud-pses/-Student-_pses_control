#include "pses_control/pses_sensor_node.hpp"
#define _USE_MATH_DEFINES
#define WHEEL_RADIUS 6.5



PsesSensor::PsesSensor() {          //Constructor
    sub_hall_counter= nh.subscribe<std_msgs::UInt8>("/uc_bridge/hall_cnt", 10, boost::bind(hall_cnt_Callback, _1, &msg_hall_counter));
    sub_hall_dt     = nh.subscribe<std_msgs::Float64>("/uc_bridge/hall_dt", 10, boost::bind(hall_dt_Callback, _1, &msg_hall_dt));
    sub_hall_dt8    = nh.subscribe<std_msgs::Float64>("/uc_bridge/hall_dt8", 10, boost::bind(hall_dt8_Callback, _1, &msg_hall_dt8));
}

//Callback function for Hall_Counter
//writes msg from Hall Coutner Topic to local Variable
void PsesSensor::hall_cnt_Callback(std_msgs::UInt8::ConstPtr hall_cnt_msg, std_msgs::UInt8* m_hall_cnt){
    *m_hall_cnt = *hall_cnt_msg;
}
void PsesSensor::hall_dt_Callback(std_msgs::Float64::ConstPtr hall_dt_msg, std_msgs::Float64* msg_hall_dt){
    *msg_hall_dt = *hall_dt_msg;
}
void PsesSensor::hall_dt8_Callback(std_msgs::Float64::ConstPtr hall_dt8_msg, std_msgs::Float64* msg_hall_dt8){
    *msg_hall_dt8 = *hall_dt8_msg;
}

//Outputs the Sensor Values and calculates the velocity
void PsesSensor::calculateVelocity(){
    ROS_INFO("Counter: %i, dt: %f, dt8: %f", msg_hall_counter.data, msg_hall_dt.data, msg_hall_dt8.data);
    velocity = msg_hall_dt.data * WHEEL_RADIUS * M_PI / 4;
    ROS_INFO("Velocity: %f", velocity);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pses_sensor_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle _nh("~");	// use this for private params
    ros::Rate loop_rate(10);
    PsesSensor sensornode;
    while (ros::ok()) {
        sensornode.calculateVelocity();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
