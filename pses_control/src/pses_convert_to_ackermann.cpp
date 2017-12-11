//include </home/pses/catkin_ws/src/pses_control/pses_control/include/pses_control/pses_convert_to_ackermann.hpp>
//#include </home/robin/catkin_ws/src/pses_control/pses_control/include/pses_control/pses_convert_to_ackermann.hpp>

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

        // use this for global access
ackermann_msgs::AckermannDriveStamped ack_msg;
//geometry_msgs::Twist m_cmd_vel;


double convert_trans_rot_vel_to_steering_angle(double v, double omega, double wheelbase){

  if (omega == 0 || v == 0){
    return 0;
  }

  double radius = v/omega;

  return atan(wheelbase/radius);
}

void cmd_callback(const geometry_msgs::Twist::ConstPtr& data){
  double wheelbase = 0.255;
  double v = data->linear.x;
  double steering = convert_trans_rot_vel_to_steering_angle(v, data->angular.z, wheelbase);

  ack_msg.header.stamp = ros::Time::now();
  ack_msg.header.frame_id = "odom"; //todo
  ack_msg.drive.steering_angle = steering;
  ack_msg.drive.speed = v;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pses_convert_to_ackermann");
    ros::NodeHandle nh;
    // ros::NodeHandle nh;
    while (ros::ok()) {
        std::string twist_cmd_topic;
        ros::Subscriber m_sub_ackermann = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(cmd_callback, _1));

        ros::Publisher m_pub_ackermann = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd_topic", 1);
        m_pub_ackermann.publish(ack_msg);

        //nh.getParam("\twist_cmd_topic", cmd_vel);
        //nh.getParam("\ackermann_cmd_topic", ackermann_cmd_topic);
        //nh.getParam("\wheelbase", 1.0);
        //nh.getParam("\frame_id", "odom");

        ros::spinOnce();
    }
    ROS_INFO("Hello world!");
    return 0;
} 
