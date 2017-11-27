//#include "pses_control/pses_control_node.hpp"
#include </home/pses/catkin_ws/src/pses_control/include/pses_control/pses_control_node.hpp>


PsesControl::PsesControl() {

    m_pub_velocity = nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
    m_pub_steering = nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

    m_sub_usr = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usr", 10, boost::bind(usrCallback, _1, &m_usr));
    m_sub_usl = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usl", 10, boost::bind(uslCallback, _1, &m_usl));
    m_sub_usf = nh.subscribe<sensor_msgs::Range>("/uc_bridge/usf", 10, boost::bind(usfCallback, _1, &m_usf));

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

void PsesControl::paramCallback(pses_control::controllerConfig &config, uint32_t level){
    ROS_INFO("Reconfigure Request: %f %f %f %f %i", config.target_value, config.kp, config.ki, config.kd, config.velocity);
    m_target_value = config.target_value;
    m_kp = config.kp;
    m_ki = config.ki;
    m_kd = config.kd;
    m_velocity_config.data = config.velocity;
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
            m_steering.data = 7869.8 * pow(steering_angle, 5) - 17042 * pow(steering_angle, 4) - 1587 * pow(steering_angle, 3) + 3098 * pow(steering_angle, 2) + 2471.8 * steering_angle - 127.6;
            ROS_INFO_STREAM(m_steering.data);
            m_time_last = time_now;
            m_e_last = m_e;

            /* limit controller output */
            if (m_steering.data > m_steering_max.data)
            {
                m_steering.data = m_steering_max.data;
            }
            else if (m_steering.data < m_steering_min.data){
                m_steering.data = m_steering_min.data;
            }
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
    ros::NodeHandle _nh("~");	// use this for private params
    ros::Rate loop_rate(10);
    PsesControl controller;
    signal(SIGINT, signalHandler);
    while (ros::ok()) {
        controller.pidControl();
        if (stop_request) {
            controller.reset();
            ros::shutdown();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
