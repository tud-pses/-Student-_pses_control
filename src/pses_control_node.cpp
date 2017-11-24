//#include "pses_control/pses_control_node.hpp"
#include </home/pses/catkin_ws/src/pses_control/include/pses_control/pses_control_node.hpp>


PsesControl::PsesControl() {

  motorCtrl =
          nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  steeringCtrl =
          nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  usrSub = nh.subscribe<sensor_msgs::Range>(
                "/uc_bridge/usr", 10, boost::bind(usrCallback, _1, &usr));
  uslSub = nh.subscribe<sensor_msgs::Range>(
              "/uc_bridge/usl", 10, boost::bind(uslCallback, _1, &usl));
  usfSub = nh.subscribe<sensor_msgs::Range>(
              "/uc_bridge/usf", 10, boost::bind(usfCallback, _1, &usf));

  dynamic_reconfigure::Server<pses_control::controllerConfig>::CallbackType f;

  f = boost::bind(&PsesControl::paramCallback, this, _1, _2);
  server.setCallback(f);

  //Kp=3000;
  //Ki=0;
  //Kd=0;
  target_value=0.4;
  esum=0;
  e=0;
  ealt=0;
}

void PsesControl::uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* usl)
{
    *usl = *uslMsg;
}

// gets called whenever a new message is availible in the input puffer
void PsesControl::usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* usf)
{
    *usf = *usfMsg;
}

// gets called whenever a new message is availible in the input puffer
void PsesControl::usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* usr)
{
    *usr = *usrMsg;
}

/*void PsesControl::signalHandler(int arg0){
  steering.data = 0;
  motor.data = 0;
  motorCtrl.publish(motor);
  steeringCtrl.publish(steering);
  ros::shutdown();
}*/

void PsesControl::paramCallback(pses_control::controllerConfig &config, uint32_t level){
  ROS_INFO("Reconfigure Request: %f %f %f %f %i %f", config.target_value, config.kp, config.ki,config.kd,config.speed, config.steeringValue);
  target_value = config.target_value;
  Kp=config.kp;
  Ki=config.ki;
  Kd=config.kd;
  speed = config.speed;
  steeringValue = config.steeringValue;
}

void PsesControl::pidControl() {
  //motor.data = 300;
  if (usr.range!=0)
  {
    if (usl.range<=0.1)
    {
      motor.data = 0;
    } else {
      motor.data = speed;
      alpha = atan (0.1/0.18);
      beta = atan((0.1+usr.range)/(0.18+usf.range));
      gamma = PI - alpha - beta;

      adeg = atan (0.18/0.1);
      bdeg = atan ((0.18+usf.range)/(0.1+usr.range));;
      cdeg = PI - adeg - bdeg;

      z1 = ((0.18+usf.range)/(sin(gamma)))*sin(beta) - sqrt(0.1*0.1+0.18*0.18);
      z2 = ((0.1+usr.range)/(sin(cdeg)))*sin(bdeg) - sqrt(0.1*0.1+0.18*0.18);
      z_eff = sin ((cdeg+(PI-gamma))/2)*((z1+z2)/2);

      //ealt=0
      e=usr.range-target_value;
      esum=esum+e;
      ros::Time time_now = ros::Time::now();
      dt = time_now.toSec() - time_last.toSec();
      steering.data = Kp*e + Ki*dt*esum + Kd*(e-ealt)/(dt);
      time_last = time_now;
      ealt=e;
      ROS_INFO_STREAM(Kp << " "<< Ki << " "<< Kd);
      //ROS_INFO_STREAM(steering.data);
      //ROS_INFO_STREAM(e << " " << Kp << " " << steering.data << " " << alpha <<  " " << beta << " " << gamma<< " " << adeg << " " << bdeg << " " << cdeg<< " " << z1 << " " << z2 << " " << z_eff);

      //steering.data = -750;
      //motor.data = 300;
      if (steering.data>=800)
      {
        steering.data=800;
      }
      else if (steering.data<=-800){
        steering.data=-800;
      }

    }
  }
  motorCtrl.publish(motor);
  steeringCtrl.publish(steering);
}
void PsesControl::steeringTest() {
  if (usl.range<=0.1){
    motor.data = 0;
  }
  else{
    motor.data = 300;
  }
  steering.data = steeringValue;
  motorCtrl.publish(motor);
  steeringCtrl.publish(steering);
}

void PsesControl::publishReset() {
  motor.data = 0;
 // steering.data = 0;
  motorCtrl.publish(motor);
  steeringCtrl.publish(steering);
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

  sleep(0);
  while (ros::ok()) {
 //   ROS_WARN("asd");
    controller.pidControl();
    //controller.steeringTest();
    if (stop_request) {
      controller.publishReset();
      ros::shutdown();
    }
    ros::spinOnce();
    loop_rate.sleep();
	}

	return 0;
}
