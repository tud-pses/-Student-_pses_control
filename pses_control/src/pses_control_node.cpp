#include "../include/pses_control/pses_control_node.hpp"

PsesControl::PsesControl() {

    ros::NodeHandle params("~");	// use this for private params

    std::string usr_topic, usl_topic, usf_topic, color_topic, depth_topic;
    params.param<std::string>("usr", usr_topic, "/uc_bridge/usr"/*"usr_filtered"*/);
    params.param<std::string>("usl", usl_topic, "/uc_bridge/usl"/*"usl_filtered"*/);
    params.param<std::string>("usf", usf_topic, "/uc_bridge/usf"/*"usf_filtered"*/);
    params.param<std::string>("color_image", color_topic, "/kinect2/hd/image_color_rect");
    params.param<std::string>("depth_image", depth_topic, "/kinect2/sd/image_depth_rect");

    m_pub_velocity = nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
    m_pub_steering = nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

    m_sub_usr = nh.subscribe<sensor_msgs::Range>(usr_topic, 10, boost::bind(usrCallback, _1, &m_usr));
    m_sub_usl = nh.subscribe<sensor_msgs::Range>(usl_topic, 10, boost::bind(uslCallback, _1, &m_usl));
    m_sub_usf = nh.subscribe<sensor_msgs::Range>(usf_topic, 10, boost::bind(usfCallback, _1, &m_usf));

    // dynamic reconfiguration
    dynamic_reconfigure::Server<pses_control::controllerConfig>::CallbackType f;
    f = boost::bind(&PsesControl::paramCallback, this, _1, _2);
    m_server.setCallback(f);

    // initialize errors for PID controller
    m_e_sum = 0;
    m_e = 0;
    m_e_last = 0;
}

void PsesControl::uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* m_usl)
{
    *m_usl = *uslMsg;
}

void PsesControl::usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* m_usf)
{
    *m_usf = *usfMsg;
}

void PsesControl::usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* m_usr)
{
    *m_usr = *usrMsg;
}

void PsesControl::driveSteeringTest(){
    m_steering.data = m_steering_config.data;
    m_velocity.data = m_velocity_config.data;

    m_pub_velocity.publish(m_velocity);
    m_pub_steering.publish(m_steering);
}

void PsesControl::paramCallback(pses_control::controllerConfig &config, uint32_t level){
    ROS_INFO("Reconfigure Request: %f %f %f %f %i", config.target_value, config.kp, config.ki, config.kd, config.velocity);
    m_target_value = config.target_value;
    m_kp = config.kp;
    m_ki = config.ki;
    m_kd = config.kd;
    m_velocity_config.data = config.velocity;
    m_steering_config.data = config.steering;
    m_steering_min.data = config.pid_min;
    m_steering_max.data = config.pid_max;
}

void PsesControl::pidControl() {
    if (m_usr.range != 0)   // filter unvalid zero values
    {
        if (m_usl.range <= 0.1)    // Mode for carrying car
        {
            m_velocity.data = 0;
        } else {    // pid controller
            m_velocity.data = m_velocity_config.data;
            m_e = m_usr.range - m_target_value;
            m_e_sum = m_e_sum + m_e;
            ros::Time time_now = ros::Time::now();
            double dt = time_now.toSec() - m_time_last.toSec();
            double steering_angle = m_kp * m_e + m_ki * dt * m_e_sum + m_kd * (m_e - m_e_last) / dt;

            // Limit for steering angle
            if (steering_angle > 0.290889)
            {
              steering_angle = 0.290889;
            }
            else if (steering_angle < -0.354883){
              steering_angle = -0.354883;
            }

            // Model for steering angle
            m_steering.data = 196814 * pow(steering_angle, 6) + 50518 * pow(steering_angle, 5) - 47550 * pow(steering_angle, 4) - 5979.7 * pow(steering_angle, 3) + 2459.5 * pow(steering_angle, 2) + 2442.1 * steering_angle + 143.78;
            m_time_last = time_now;
            m_e_last = m_e;
        }
    }
    m_pub_velocity.publish(m_velocity);
    m_pub_steering.publish(m_steering);
}

void PsesControl::reset() {
    m_velocity.data = 0;
    m_steering.data = 0;
    m_pub_velocity.publish(m_velocity);
    m_pub_steering.publish(m_steering);
}

void signalHandler(int sig) {
    stop_request = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pses_control_node", ros::init_options::NoSigintHandler);
    PsesControl controller;
    ros::Rate loop_rate(10);
    signal(SIGINT, signalHandler);
    while (ros::ok()) {
        controller.pidControl();
        //controller.driveSteeringTest();
        if (stop_request) {
            controller.reset();
            ros::shutdown();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
