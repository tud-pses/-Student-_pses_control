//#include "pses_control/pses_control_node.hpp"
#include </home/pses/catkin_ws/src/pses_control/pses_control/include/pses_control/pses_control_node.hpp>


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

    dynamic_reconfigure::Server<pses_control::controllerConfig>::CallbackType f;
    f = boost::bind(&PsesControl::paramCallback, this, _1, _2);
    m_server.setCallback(f);

    // default pid values set by reconfigure
    //m_kp = 1000;
    //m_ki = 0;
    //m_kd = 5000;
    //m_target_value = 0.4;
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
    m_velocity.data = 300;

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
    if (m_usr.range != 0)
    {
        if (m_usl.range <= 0.1)
        {
            m_velocity.data = 0;
        } else {
            m_velocity.data = m_velocity_config.data;

            /* compute closest distance from front corner to wall */
            double alpha0, beta0, gamma0, alpha1, beta1, gamma1, z1, z2, z_eff;
            alpha0 = atan (0.1 / 0.18);
            beta0 = atan((0.1 + m_usr.range) / (0.18 + m_usf.range));
            gamma0 = PI - alpha0 - beta0;

            alpha1 = atan (0.18 / 0.1);
            beta1 = atan ((0.18 + m_usf.range) / (0.1 + m_usr.range));;
            gamma1 = PI - alpha1 - beta1;

            z1 = ((0.18 + m_usf.range) / (sin(gamma0))) * sin(beta0) - sqrt(0.1 * 0.1 + 0.18 * 0.18);
            z2 = ((0.1 + m_usr.range) / (sin(gamma1))) * sin(beta1) - sqrt(0.1 * 0.1 + 0.18 * 0.18);
            z_eff = sin((gamma1 + (PI - gamma0)) / 2) * ((z1 + z2) / 2);

            /* pid controller */
            m_e = m_usr.range - m_target_value;
            m_e_sum = m_e_sum + m_e;
            ros::Time time_now = ros::Time::now();
            double dt = time_now.toSec() - m_time_last.toSec();
            double steering_angle = m_kp * m_e + m_ki * dt * m_e_sum + m_kd * (m_e - m_e_last) / dt;

            // Limit for steering angle
            /*if (steering_angle > 0.3316)
            {
              steering_angle = 0.3316;
            }
            else if (steering_angle < -0.3374){
              steering_angle = -0.3374;
            }*/

            if (steering_angle > 0.290889)
            {
              steering_angle = 0.290889;
            }
            else if (steering_angle < -0.354883){
              steering_angle = -0.354883;
            }

            //m_steering.data = m_kp * m_e + m_ki * dt * m_e_sum + m_kd * (m_e - m_e_last) / dt;
            //m_steering.data = 7869.8 * pow(steering_angle, 5) - 17042 * pow(steering_angle, 4) - 1587 * pow(steering_angle, 3) + 3098 * pow(steering_angle, 2) + 2471.8 * steering_angle - 127.6;
            m_steering.data = 196814 * pow(steering_angle, 6) + 50518 * pow(steering_angle, 5) - 47550 * pow(steering_angle, 4) - 5979.7 * pow(steering_angle, 3) + 2459.5 * pow(steering_angle, 2) + 2442.1 * steering_angle + 143.78;
            m_time_last = time_now;
            m_e_last = m_e;

            /* limit controller output */
            /*if (m_steering.data > m_steering_max.data)
            {
                m_steering.data = m_steering_max.data;
            }
            else if (m_steering.data < m_steering_min.data){
                m_steering.data = m_steering_min.data;
            }*/
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
