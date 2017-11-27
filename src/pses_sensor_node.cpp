#include </home/pses/catkin_ws/src/pses_control/include/pses_control/pses_sensor_node.hpp>
#define _USE_MATH_DEFINES
#define WHEEL_RADIUS 5



PsesSensor::PsesSensor() {          //Constructor
    sub_hall_counter= nh.subscribe<std_msgs::UInt8>("/uc_bridge/hall_cnt", 10, boost::bind(hall_cnt_Callback, _1, &msg_hall_counter));
    sub_hall_dt     = nh.subscribe<std_msgs::UInt8>("/uc_bridge/hall_dt", 10, boost::bind(hall_dt_Callback, _1, &msg_hall_dt));
    sub_hall_dt8    = nh.subscribe<std_msgs::UInt8>("/uc_bridge/hall_dt8", 10, boost::bind(hall_dt8_Callback, _1, &msg_hall_dt8));

}

//Callback function for Hall_Counter
//writes msg from Hall Coutner Topic to local Variable
void PsesControl::hall_cnt_Callback(std_msgs::UInt8 hall_cnt_msg, std_msgs::UInt8* m_hall_cnt){
    *m_hall_cnt = *hall_cnt_msg;
}
void PsesControl::hall_dt_Callback(std_msgs::Float64 hall_dt_msg, std_msgs::Float64* msg_hall_dt){
    *msg_hall_dt = *hall_dt_msg;
}
void PsesControl::hall_dt8_Callback(std_msgs::Float64 hall_dt8_msg, std_msgs::Float64* msg_hall_dt8){
    *msg_hall_dt8 = *hall_dt8_msg;
}

//Outputs the Sensor Values and calculates the velocity
void PSESControl::calculateVelocity(){
    ROS_INFO("Hal Sensor: Counter: %i, dt: %f, dt8: %f", msg_hall_counter, msg_hall_dt, msg_hall_dt8);
    float velocity = msg_hall_dt8 * WHEEL_RADIUS*M_PI/4;
    ROSINFO("Velocity: %f", velocity);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pses_sensor_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle _nh("~");	// use this for private params
    ros::Rate loop_rate(10);

    PsesSensor sensornode;
    while (ros::ok()) {
        calculateVelocity();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
