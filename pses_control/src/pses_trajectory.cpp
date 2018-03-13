#include </home/pses/catkin_ws/src/pses_control/pses_control/include/pses_control/pses_trajectory.hpp>

PsesTrajectory::PsesTrajectory() {
    ros::NodeHandle params("~");	// use this for private params

    m_pub_velocity = nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
    m_pub_steering = nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

    //m_sub_ackermann_cmd = nh.subscribe<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd_topic", 10, boost::bind(ackermannCmdCallback, _1, &m_ack_steering, &m_ack_vel));
    m_sub_ackermann_cmd = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, boost::bind(cmdCallback, _1, &m_ack_steering, &m_ack_vel));
    //m_sub_follow_goal = nh.subscribe<geometry_msgs::PoseStamped>("/follow_goal", 10, boost::bind(followGoalCallback, _1,&m_follow_goal));
    m_sub_amcl_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, boost::bind(amclPoseCallback, _1, &m_amcl_pose));
    m_pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);

    dynamic_reconfigure::Server<pses_control::controllerConfig>::CallbackType f;
    f = boost::bind(&PsesTrajectory::paramCallback, this, _1, _2);
    m_server.setCallback(f);
    m_goal_counter=0;
    m_problem_counter=0;
    m_old_ack_velocity=0;
    m_problem_detected = false;
}
/*
void PsesTrajectory::ackermannCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& ackermannCmdMsg, double* m_ack_steering, double* m_ack_vel)
{
    //ROS_INFO("Ackermann Command : steering angle = %f - speed = %f", ackermannCmdMsg->drive.steering_angle, ackermannCmdMsg->drive.speed);
    *m_ack_steering=ackermannCmdMsg->drive.steering_angle;
    *m_ack_vel=ackermannCmdMsg->drive.speed;
}
*/
void PsesTrajectory::cmdCallback(const geometry_msgs::Twist::ConstPtr& data, double* m_ack_steering, double* m_ack_vel){
  *m_ack_steering = data->angular.z;
  *m_ack_vel = data->linear.x;

}

/*void PsesTrajectory::followGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& followGoalMsg, geometry_msgs::PoseStamped* m_follow_goal){
    *m_follow_goal = *followGoalMsg;
}*/

void PsesTrajectory::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose, geometry_msgs::PoseWithCovarianceStamped* m_amcl_pose){
    *m_amcl_pose = *amcl_pose;
}

void PsesTrajectory::publishGoal(){
    double goal_orient_z, goal_orient_w, distance_to_goal;
    double pose_pos_x, pose_pos_y;

    pose_pos_x = m_amcl_pose.pose.pose.position.x;
    pose_pos_y = m_amcl_pose.pose.pose.position.y;

    if (m_goal_counter == 0){
        distance_to_goal = 0;
    }
    else{
        distance_to_goal = sqrt(pow((m_goal_pos_x-pose_pos_x),2)+pow((m_goal_pos_y-pose_pos_y),2));
    }

    //ROS_INFO("pose x : %f - pose y : %f - distance : %f - goal_pose_x : %f - goal_pos_y : %f", pose_pos_x, pose_pos_y, distance_to_goal, m_goal_pos_x, m_goal_pos_y);

    if (distance_to_goal < 1.5){
        ROS_INFO("counter: %d", m_goal_counter);
        switch (m_goal_counter) {
        case 0:
             m_goal_pos_x = 20.0;
             m_goal_pos_y = 0.6;
             goal_orient_z = 0.0;
             goal_orient_w = 1.0;
             sleep(2);
             break;
        case 1:
            m_goal_pos_x = 25.5;
            m_goal_pos_y = 1.0;
            goal_orient_z = 0.0;
            goal_orient_w = 1.0;
            break;
        case 2:
            m_goal_pos_x = 27.65;
            m_goal_pos_y = -2.5;
            goal_orient_z = -0.707;
            goal_orient_w = 0.707;
            break;
        case 3:
            m_goal_pos_x = 27.9;
            m_goal_pos_y = -5.7;
            goal_orient_z = -0.707;
            goal_orient_w = 0.707;
            break;
        case 4:
            m_goal_pos_x = 23.5;
            m_goal_pos_y = -7.1;
            goal_orient_z = 1.0;
            goal_orient_w = 0.0;
            break;
        case 5:
            m_goal_pos_x = 14.5;
            m_goal_pos_y = -7.6;
            goal_orient_z = 1.0;
            goal_orient_w = 0.0;
            break;
        case 6:
            m_goal_pos_x = 12.9;
            m_goal_pos_y = -4.5;
            goal_orient_z = 0.707;
            goal_orient_w = 0.707;
            break;
        case 7:
            m_goal_pos_x = 13.0;
            m_goal_pos_y = -1.7;
            goal_orient_z = 0.707;
            goal_orient_w = 0.707;
            break;
        case 8:
            m_goal_pos_x = 20.0;
            m_goal_pos_y = 0.6;
            goal_orient_z = 0.0;
            goal_orient_w = 1.0;
            break;
        default: //TODO
            stop_request = true;
            m_goal_pos_x = pose_pos_x;
            m_goal_pos_y = pose_pos_y;
            goal_orient_z = 0.0;
            goal_orient_w = 1.0;
            break;
        }
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position.x = m_goal_pos_x;
        goal_msg.pose.position.y = m_goal_pos_y;
        goal_msg.pose.position.z = 0.0;
        goal_msg.pose.orientation.x = 0.0;
        goal_msg.pose.orientation.y = 0.0;
        goal_msg.pose.orientation.z = goal_orient_z;
        goal_msg.pose.orientation.w = goal_orient_w;

        m_pub_goal.publish(goal_msg);

        m_goal_counter++;
    }
}

void PsesTrajectory::driveTrajectory(){

    // Limit for steering angle
    if (m_ack_steering < -0.290889)
    {
      m_ack_steering = -0.290889;
    }
    else if (m_ack_steering > 0.354883){
      m_ack_steering = 0.354883;
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
    m_steering.data = (196814 * pow(m_ack_steering, 6) - 50518 * pow(m_ack_steering, 5) - 47550 * pow(m_ack_steering, 4) + 5979.7 * pow(m_ack_steering, 3) + 2459.5 * pow(m_ack_steering, 2) - 2442.1 * m_ack_steering + 143.78);

    //Velocity
    if (m_ack_vel < - 0.825872){
        m_velocity.data=-500;
    }
    else if (m_ack_vel > 2.013835){
        m_velocity.data=1000;
    }
    else if (m_ack_vel < 0 && m_ack_vel >= - 0.825872){
        m_velocity.data=477.52 * m_ack_vel - 104.62;
    }
    else if (m_ack_vel <= 2.013835 && m_ack_vel > 0){
        m_velocity.data=439.8 * m_ack_vel + 116.14;
    }
    else{
        m_velocity.data = 0;
    }

    //ROS_INFO("velocities : velocity_real = %d - velocity_ack = %f", m_velocity.data, m_ack_vel);

    m_pub_velocity.publish(m_velocity);
    m_pub_steering.publish(m_steering);
}

void PsesTrajectory::clearCostmap() {
    bool execute_clear = false;
    ros::Duration duration_between_problem;

    if (m_old_ack_velocity * m_ack_vel <= 0 && m_problem_detected == false){
        m_begin_change = ros::Time::now();
        m_problem_detected = true;
        m_problem_counter++;
    }
    else if (m_old_ack_velocity * m_ack_vel <= 0 && m_problem_detected == true){
        duration_between_problem = ros::Time::now() - m_begin_change;
        if (duration_between_problem.toSec() < 2){
            m_problem_counter++;
            if (m_problem_counter >= 13){
                execute_clear = true;
                m_problem_detected = false;
                m_problem_counter = 0;
            }
        }
        else {
            m_problem_counter = 0;
            m_problem_detected = false;
        }
    }
    else {
        duration_between_problem = ros::Time::now() - m_begin_change;
        if (duration_between_problem.toSec() > 2){
            m_problem_detected = false;
            m_problem_counter = 0;
        }
    }

    if (execute_clear) {
        tf::TransformListener tf(ros::Duration(10));
        costmap_2d::Costmap2DROS global_costmap("global_costmap", tf);
        costmap_2d::Costmap2DROS local_costmap("local_costmap", tf);
        clear_costmap_recovery::ClearCostmapRecovery ccr;
        ccr.initialize("my_clear_costmap_recovery", &tf, &global_costmap, &local_costmap);

        ccr.runBehavior();
    }
    m_old_ack_velocity = m_ack_vel;
    ROS_INFO("problem counter = %d , m_problem_detected = %d, duration = %f , begin = %f", m_problem_counter, m_problem_detected, duration_between_problem.toSec(),m_begin_change.toSec());
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
        controller.publishGoal();
        controller.driveTrajectory();
        //controller.clearCostmap();
        if (stop_request) {
            controller.reset();
            ros::shutdown();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
