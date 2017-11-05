#include "pses_control/pses_control_node.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "pses_control_node");
	ros::NodeHandle nh;	// use this for global access
	ros::NodeHandle _nh("~");	// use this for private params
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		ROS_WARN("asd");
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
