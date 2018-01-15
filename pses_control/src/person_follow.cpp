#include <ros/ros.h>
#include "pses_control/person_follow_detection.hpp"
#include "pses_control/person_follow_tracking.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>

PersonFollowDetection detector;
PersonFollowTracking tracker;
Rect2d bbox;
ros::Subscriber sub_color_image_;

void detectOrTrack(Mat frame) {
    if(bbox.tl()==bbox.br()) {
        bbox=detector.detect(frame);
    } else {
        bbox=tracker.track(frame, bbox);
    }

}

void colorImageCallback(const sensor_msgs::ImageConstPtr& color_image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
        detectOrTrack(cv_ptr->image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error: %s", e.what());
        // return;
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "person_follow");
    ros::NodeHandle nh;
    ros::NodeHandle params("~");
    std::string color_image_topic;
    params.param<std::string>("image", color_image_topic, "/kinect2/hd/image");
    //params.param<bool>("show", show_, false);
    sub_color_image_ = nh.subscribe(color_image_topic, 50, &colorImageCallback);    // optimally image is resized

    ros::Rate loop_rate(10);
    while (ros::ok()) { //node_obj.nh.ok()
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
