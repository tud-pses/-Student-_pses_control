#include </home/pses/catkin_ws/src/pses_control/include/pses_control/pses_sensor_node.hpp>


PsesSensor::PsesSensor() {          //Constructor
    m_sub_hall_cnt= nh.subscribe<std_msgs::UInt8>("/uc_bridge/hall_cnt", 10, boost::bind(hall_cntCallback, _1, &m_hall_cnt));
}

//Callback function for Hall_Counter
//writes msg from Hall Coutner Topic to local Variable
void PsesControl::hall_cntCallback(std_msgs::UInt8 hall_cnt_msg, std_msgs::UInt8* m_hall_cnt){
    *m_hall_cnt = *hall_cnt_msg;
}

//Outputs the Sensor Values and calculates the velocity
void PSESControl::calculateVelocity(){
    ROS_INFO("hall_cnt: %i", m_hall_cnt);
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
