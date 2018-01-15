#include "pses_control/pses_sensor_node.hpp"
#define _USE_MATH_DEFINES



pses_sensor::pses_sensor() {          //Constructor
    sub_hall_counter= nh.subscribe<std_msgs::UInt8>("/uc_bridge/hall_cnt", 10, boost::bind(hall_cnt_Callback, _1, &msg_hall_counter));
    sub_hall_dt     = nh.subscribe<std_msgs::Float64>("/uc_bridge/hall_dt", 10, boost::bind(hall_dt_Callback, _1, &msg_hall_dt));
    sub_hall_dt8    = nh.subscribe<std_msgs::Float64>("/uc_bridge/hall_dt8", 10, boost::bind(hall_dt8_Callback, _1, &msg_hall_dt8));
    sub_usr = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usr", 10, boost::bind(usrCallback, _1, &m_usr));
    sub_usl = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usl", 10, boost::bind(uslCallback, _1, &m_usl));
    sub_usf = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usf", 10, boost::bind(usfCallback, _1, &m_usf));

    pub_usr = nh.advertise<sensor_msgs::Range>("sensor/usr",10);
    m_pub_velocity = nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
    m_pub_steering = nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);
    m_pub_speed = nh. advertise<std_msgs::Float64>("/sensornode/speed",10);
    m_pub_distance = nh.advertise<std_msgs::Float64>("/sensornode/distance",10);
    m_pub_avg_speed = nh.advertise<std_msgs::Float64>("/sensornode/avg_speed",10);

    dynamic_reconfigure::Server<pses_control::sensorConfig>::CallbackType f;
    f = boost::bind(&pses_sensor::param_callback, this, _1, _2);
    server.setCallback(f);
    data_count=0;
    avg_speed.data=0;
}

//Callback function for Hall_Counter
//writes msg from Hall Coutner Topic to local Variable
void pses_sensor::hall_cnt_Callback(std_msgs::UInt8::ConstPtr hall_cnt_msg, std_msgs::UInt8* m_hall_cnt){
    *m_hall_cnt = *hall_cnt_msg;
}
void pses_sensor::hall_dt_Callback(std_msgs::Float64::ConstPtr hall_dt_msg, std_msgs::Float64* msg_hall_dt){
    *msg_hall_dt = *hall_dt_msg;
}
void pses_sensor::hall_dt8_Callback(std_msgs::Float64::ConstPtr hall_dt8_msg, std_msgs::Float64* msg_hall_dt8){
    *msg_hall_dt8 = *hall_dt8_msg;
}

//Callback functions for ultra sonic sensor
void pses_sensor::usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* m_usr){
    if(usrMsg->range != 0){
       float output_old = m_usr->range;
       *m_usr = *usrMsg;
       m_usr->range = pses_sensor::lowpass(usrMsg->range, output_old, 7.0);
    }
}
void pses_sensor::uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* m_usl){
    if(uslMsg->range !=0){
        *m_usl = *uslMsg;
    }
}
void pses_sensor::usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* m_usf){
        if(usfMsg->range != 0){
            *m_usf = *usfMsg;
        }
}

//Callback function for dynamic reconfigure
void pses_sensor::param_callback(pses_control::sensorConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %d", config.velocity, config.steering);
  target_speed.data = config.velocity;
  target_steering_angle.data = config.steering;
}
float pses_sensor::lowpass(float input, float output_old, float tau){
    return ((input+output_old*tau)/(tau+1));
}
//Outputs the Sensor Values and calculates the velocity
void pses_sensor::calculateVelocity(){
    //velocity.data =  WHEEL_RADIUS * M_PI / 4 / msg_hall_dt.data;
    velocity.data = DRIVEN_DISTANCE_PER_TICK / msg_hall_dt.data;
    //ROS_INFO("Velocity PSES: %f", velocity.data);
    if(data_count ==0){
        avg_speed.data = velocity.data;
        data_count++;
    }
    else if(data_count != 0 && data_count < 400){
        data_count++;
        avg_speed.data += velocity.data;
    }
    else if(data_count = 400){
        avg_speed.data /= data_count;
        ROS_INFO("AVG_Speed: %f", avg_speed.data);
        m_pub_avg_speed.publish(avg_speed);
        data_count =0;
    }
}

void pses_sensor::publish_sensor_data(){
    pub_usr.publish(m_usr);
    m_pub_speed.publish(velocity);
}

void pses_sensor::set_powertrain(){
    m_pub_velocity.publish(target_speed);
    m_pub_steering.publish(target_steering_angle);
}

void pses_sensor::reset(){
    target_speed.data=0;
    target_steering_angle.data=0;
    m_pub_velocity.publish(target_speed);
    m_pub_steering.publish(target_steering_angle);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pses_sensor_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle _nh("~");	// use this for private params
    ros::Rate loop_rate(100);
    pses_sensor sensornode;
    while (ros::ok()) {
        sensornode.calculateVelocity();
        sensornode.publish_sensor_data();
        sensornode.set_powertrain();
        ros::spinOnce();
        loop_rate.sleep();
    }
    sensornode.reset();

    return 0;
}
