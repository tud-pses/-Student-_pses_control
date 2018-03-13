#include "pses_control/pses_sensor_node.hpp"

pses_sensor::pses_sensor() {          //Constructor
    // subscribers of the hall sensor
    sub_hall_counter= nh.subscribe<std_msgs::UInt8>("/uc_bridge/hall_cnt", 10, boost::bind(hall_cnt_Callback, _1, &msg_hall_counter));
    sub_hall_dt     = nh.subscribe<std_msgs::Float64>("/uc_bridge/hall_dt", 10, boost::bind(hall_dt_Callback, _1, &msg_hall_dt));
    sub_hall_dt8    = nh.subscribe<std_msgs::Float64>("/uc_bridge/hall_dt8", 10, boost::bind(hall_dt8_Callback, _1, &msg_hall_dt8));
    sub_motor_speed_ = nh.subscribe<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1, boost::bind(motorSpeedCallback, _1, &msg_motor_speed));
    // subscribers for ultra sonic range sensors
    sub_usr = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usr", 10, boost::bind(usrCallback, _1, &m_usr, &usr_tau));

    pub_usr = nh.advertise<sensor_msgs::Range>("/sensornode/usr",10);
    //publisher of velocity data
    pub_distance = nh.advertise<std_msgs::Float64>("/sensornode/distance",10);
    pub_avg_speed = nh.advertise<std_msgs::Float64>("/sensornode/avg_speed",10);
    pub_wheel_odom = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/sensornode/wheel_odom",10);
    // reconfigure server
    dynamic_reconfigure::Server<pses_control::sensorConfig>::CallbackType f;
    f = boost::bind(&pses_sensor::paramCallback, this, _1, _2);
    server.setCallback(f);

    //set default value to variables
    data_count=0;
    avg_speed.data=0.0;
}

//Callback function for Hall_Counter
void pses_sensor::hall_cnt_Callback(std_msgs::UInt8::ConstPtr hall_cnt_msg, std_msgs::UInt8* m_hall_cnt){
    *m_hall_cnt = *hall_cnt_msg;
}
void pses_sensor::hall_dt_Callback(std_msgs::Float64::ConstPtr hall_dt_msg, std_msgs::Float64* msg_hall_dt){
    *msg_hall_dt = *hall_dt_msg;
}
void pses_sensor::hall_dt8_Callback(std_msgs::Float64::ConstPtr hall_dt8_msg, std_msgs::Float64* msg_hall_dt8){
    *msg_hall_dt8 = *hall_dt8_msg;
}

//Callback function for ultra sonic sensor
void pses_sensor::usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* m_usr, float* usr_tau){
    if(usrMsg->range != 0){                        //filters invalid zero values
       float output_old = m_usr->range;
       *m_usr = *usrMsg;
       m_usr->range = pses_sensor::lowpass(usrMsg->range, output_old, *usr_tau);
    }
}

void pses_sensor::motorSpeedCallback(std_msgs::Int16::ConstPtr motor_speed_msg, std_msgs::Int16* msg_motor_speed){
    *msg_motor_speed = *motor_speed_msg;
}

void pses_sensor::paramCallback(pses_control::sensorConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request:\n usr_tau: %f", config.tau);
  usr_tau = config.tau;
}

//lowpass filter function
float pses_sensor::lowpass(float input, float output_old, float tau){
    return ((input+output_old*tau)/(tau+1));
}

void pses_sensor::calculateVelocity(){
    if(msg_motor_speed.data >0){  //direction of driving
        velocity.data = DRIVEN_DISTANCE_PER_TICK / msg_hall_dt.data;
    }
    else if(msg_motor_speed.data <0){
            velocity.data = DRIVEN_DISTANCE_PER_TICK / msg_hall_dt.data * (-1);
        }
    else {
        velocity.data = 0;
    }
        driven_distance.data = DRIVEN_DISTANCE_PER_TICK * msg_hall_dt.data;
        //calculate average speed over 5 seconds
        if(data_count ==0){
            avg_speed.data = velocity.data;
            data_count++;
        }
        else if(data_count != 0 && data_count < 500){
            data_count++;
            avg_speed.data += velocity.data;
        }
        else if(data_count = 500){
            avg_speed.data /= data_count;
            pub_avg_speed.publish(avg_speed);
            data_count =0;
        }
    }

void pses_sensor::publishSensorData(){
   pub_usr.publish(m_usr);
   pub_distance.publish(driven_distance);
   wheel_odom.twist.twist.linear.x = velocity.data;
   wheel_odom.header.frame_id = "base_link";
   wheel_odom.header.stamp = ros::Time::now();
   pub_wheel_odom.publish(wheel_odom);
}


//signal handler to recognize Strg-C abort request of the programm
void signalHandler(int sig) {
    stop_request = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pses_sensor_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle _nh("~");	// use this for private params
    ros::Rate loop_rate(100);
    pses_sensor sensornode;
    signal(SIGINT, signalHandler);
    while (ros::ok()) {
        sensornode.calculateVelocity();
        sensornode.publishSensorData();
        if (stop_request) {
            ros::shutdown();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
