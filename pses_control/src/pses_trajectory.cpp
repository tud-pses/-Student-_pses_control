#include </home/pses/catkin_ws/src/pses_control/pses_control/include/pses_control/pses_trajectory.hpp>

PsesTrajectory::PsesTrajectory() {
    ros::NodeHandle params("~");	// use this for private params

    m_pub_velocity = nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
    m_pub_steering = nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

    m_sub_ackermann_cmd = nh.subscribe<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd_topic", 10, boost::bind(ackermannCmdCallback, _1, &m_ack_steering, &m_ack_vel));
    m_sub_follow_goal = nh.subscribe<geometry_msgs::PoseStamped>("/follow_goal", 10, boost::bind(followGoalCallback, _1,&m_follow_goal));
    m_pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);

    dynamic_reconfigure::Server<pses_control::controllerConfig>::CallbackType f;
    f = boost::bind(&PsesTrajectory::paramCallback, this, _1, _2);
    m_server.setCallback(f);
}

void PsesTrajectory::ackermannCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& ackermannCmdMsg, double* m_ack_steering, double* m_ack_vel)
{
    //ROS_INFO("Ackermann Command : steering angle = %f - speed = %f", ackermannCmdMsg->drive.steering_angle, ackermannCmdMsg->drive.speed);
    *m_ack_steering=ackermannCmdMsg->drive.steering_angle;
    *m_ack_vel=ackermannCmdMsg->drive.speed;
}

void PsesTrajectory::followGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& followGoalMsg, geometry_msgs::PoseStamped* m_follow_goal){
    *m_follow_goal = *followGoalMsg;
}

void PsesTrajectory::publishGoal(){

    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = 1.5;
    goal_msg.pose.position.y = 0;
    goal_msg.pose.position.z = 0;
    goal_msg.pose.orientation.x = 0;
    goal_msg.pose.orientation.y = 0;
    goal_msg.pose.orientation.z = 0;
    goal_msg.pose.orientation.w = 1;

    m_pub_goal.publish(goal_msg);
}

void PsesTrajectory::driveTrajectory(){

    // Limit for steering angle
    if (m_ack_steering > 0.290889)
    {
      m_ack_steering = 0.290889;
    }
    else if (m_ack_steering < -0.354883){
      m_ack_steering = -0.354883;
    }

    /*if (m_ack_vel > 0.5)
    {
      m_velocity.data = 300;
    }
    else if (m_ack_vel < -0.5){
      m_velocity.data = -300;
    }
    else{
        m_velocity.data = m_ack_vel*600;
    }*/

    //m_steering.data = 7869.8 * pow(m_ack_steering, 5) - 17042 * pow(m_ack_steering, 4) - 1587 * pow(m_ack_steering, 3) + 3098 * pow(m_ack_steering, 2) + 2471.8 * m_ack_steering - 127.6;
    m_steering.data = 196814 * pow(m_ack_steering, 6) + 50518 * pow(m_ack_steering, 5) - 47550 * pow(m_ack_steering, 4) - 5979.7 * pow(m_ack_steering, 3) + 2459.5 * pow(m_ack_steering, 2) + 2442.1 * m_ack_steering + 143.78;

    //Velocity
    if (m_ack_vel < - 0.825872){
        m_velocity.data=-500;
    }
    else if (m_ack_vel > 2.013835){
        m_velocity.data=1000;
    }
    else if (m_ack_vel <= 0 && m_ack_vel > - 0.825872){
        m_velocity.data=477.52 * m_ack_vel - 104.62;
    }
    else if (m_ack_vel <= 2.013835 && m_ack_vel > 0){
        m_velocity.data=439.8 * m_ack_vel + 116.14;
    }
    else{
        m_velocity.data = 0;
    }

    //ROS_INFO("Ackermann Command2 : steering angle = %d - speed = %d", m_steering.data, m_velocity.data);

    //m_pub_velocity.publish(m_velocity);
    //m_pub_steering.publish(m_steering);
    publishGoal();
}

void PsesTrajectory::reset() {
    m_velocity.data = 0;
    m_steering.data = 0;
    m_pub_velocity.publish(m_velocity);
    m_pub_steering.publish(m_steering);
}

void signalHandler(int sig) {
    stop_request = true;
}

void PsesTrajectory::paramCallback(pses_control::controllerConfig &config, uint32_t level){
    /*ROS_INFO("Reconfigure Request: %f %f %f %f %i", config.target_value, config.kp, config.ki, config.kd, config.velocity);
    m_target_value = config.target_value;
    m_kp = config.kp;
    m_ki = config.ki;
    m_kd = config.kd;
    m_velocity_config.data = config.velocity;
    m_steering_config.data = config.steering;
    m_steering_min.data = config.pid_min;
    m_steering_max.data = config.pid_max;*/
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "pses_trajectory", ros::init_options::NoSigintHandler);
    PsesTrajectory controller;
    ros::Rate loop_rate(10);
    signal(SIGINT, signalHandler);

    while (ros::ok()) {
        controller.driveTrajectory();
        if (stop_request) {
            controller.reset();
            ros::shutdown();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
