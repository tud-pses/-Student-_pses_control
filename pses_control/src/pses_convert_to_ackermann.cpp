#include </home/pses/catkin_ws/src/pses_control/pses_control/include/pses_control/pses_convert_to_ackermann.hpp>

/*void geometryCallback(geometry_msgs::Twist::ConstPtr cmdVelMsg, geometry_msgs::Twist* m_cmd_vel)
{
    *m_cmd_vel = *cmdVelMsg;
}

double convert_trans_rot_vel_to_steering_angle(double v, double omega, double wheelbase){

  if (omega == 0 || v == 0){
    return 0;
  }

  double radius = v/omega;

  return atan(wheelbase/radius);
}
*/
int main(int argc, char **argv)
{
  return 0;
}
  /*
  ros::init(argc, argv, "pses_convert_to_ackermann");
  ros::NodeHandle nh;

  ros::Publisher m_pub_ackermann = nh.advertise<ackermann_msgs::AckermannDriveStamped>("cmd_vel_ackermann_msg", 1);
  ros::Subscriber m_sub_ackermann = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, boost::bind(geometryCallback, _1, &m_cmd_vel));

  //m_pub_ackermann.publish(ack_msg);
  std::string twist_cmd_topic, ackermann_cmd_topic, wheelbase, frame_id;
  nh.getParam("\twist_cmd_topic", twist_cmd_topic);
  nh.getParam("\ackermann_cmd_topic", ackermann_cmd_topic);
  //nh.getParam("\wheelbase", wheelbase);
  nh.getParam("\frame_id", frame_id);


//
  //m_sub_ackermann.subscriber(twist_cmd_topic);
  m_pub_ackermann.publish(ackermann_cmd_topic);

  //extern double wheelbase;
  //extern double ackermann_cmd_topic;
  //extern double frame_id;
  //extern double m_pub_ackermann;

  ackermann_msgs::AckermannDriveStamped ack_msg;
  //double wheelbase;
  double v = m_cmd_vel.linear.x;
  double steering = convert_trans_rot_vel_to_steering_angle(v,m_cmd_vel.angular.z,wheelbase);

  ack_msg.header.stamp = ros::Time::now();
  ack_msg.header.frame_id = ""; //todo
  ack_msg.drive.steering_angle = steering;
  ack_msg.drive.speed = v;


  ros::spinOnce();

  ROS_INFO("Hello world!");
}
*/
