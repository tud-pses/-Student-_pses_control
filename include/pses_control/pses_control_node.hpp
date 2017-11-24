#ifndef PSES_CONTROL_NODE_HPP
#define PSES_CONTROL_NODE_HPP
#define PI 3.14159265

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <stdlib.h>
#include <signal.h>
#include <dynamic_reconfigure/server.h>
#include <pses_control/controllerConfig.h>
#include <math.h>

class PsesControl {

public:
  PsesControl();
  static void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* usl);

  // gets called whenever a new message is availible in the input puffer
  static void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* usf);


  // gets called whenever a new message is availible in the input puffer
  static void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* usr);

  void publishReset();

  void paramCallback(pses_control::controllerConfig &config, uint32_t level);

  void pidControl();

  // Variables
  ros::NodeHandle nh;	// use this for global access
  sensor_msgs::Range usr, usf, usl;
  std_msgs::Int16 motor, steering;
  double Kp, Ki, Kd, dt;
  double e, esum, ealt;
  double target_value, speed;
  double alpha, beta, gamma, adeg, bdeg, cdeg, z1, z2, z_eff;

  double steeringValue;
  void steeringTest();

  ros::Time time_last = ros::Time::now();


  // Subscriber
  ros::Subscriber usrSub;
  ros::Subscriber uslSub;
  ros::Subscriber usfSub;
  // Publisher
  ros::Publisher motorCtrl;
  ros::Publisher steeringCtrl;

private:
    dynamic_reconfigure::Server<pses_control::controllerConfig> server;

};

void signalHandler(int sig);
bool stop_request = false;


#endif //PSES_CONTROL_NODE_HPP
