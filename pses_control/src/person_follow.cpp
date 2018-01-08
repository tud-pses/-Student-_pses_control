#include <ros/ros.h>
#include "pses_control/person_follow_detection.hpp"
#include "pses_control/person_follow_tracking.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "person_follow");
    PersonFollowDetection detector;
    ros::Rate loop_rate(10);
    while (ros::ok()) { //node_obj.nh.ok()
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
