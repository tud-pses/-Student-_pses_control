#include <ros/ros.h>
#include "pses_control/person_follow_detection.hpp"
#include "pses_control/person_follow_tracking.hpp"
#include "pses_control/person_follow_clustering.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>

PersonFollowDetection detector_;
PersonFollowTracking tracker_;
PersonFollowClustering cluster_;
Rect2d bbox_;
ros::Subscriber sub_color_image_;
ros::Subscriber sub_laser_scan_;
int tracking_id_;
int cnt_ = 0;

void detectOrTrack(Mat frame) {
    ++cnt_;
    if(bbox_.tl() == bbox_.br() || cnt_ > 100) {
        ROS_INFO_STREAM("detecting...");
        bbox_ = detector_.detect(frame);
        cnt_ = 0;
        tracker_.setInit(false);
    } else {
        ROS_INFO_STREAM("tracking...");
        bbox_ = tracker_.track(frame, bbox_);
    }
}

void colorImageCallback(const sensor_msgs::ImageConstPtr& color_image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
        Mat manip_image;
        Size manip_image_size(300, 300);
        resize(cv_ptr->image, manip_image, manip_image_size);
        detectOrTrack(manip_image);
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
    params.param<std::string>("image", color_image_topic, "/kinect2/hd/image_color");
    params.param<int>("tracker", tracking_id_, 4);
    //params.param<bool>("show", show_, false);
    tracker_.initTracker(tracking_id_);
    sub_color_image_ = nh.subscribe(color_image_topic, 10, &colorImageCallback);
    sub_laser_scan_ = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &PersonFollowClustering::scanCallback, &cluster_);

    ros::Rate loop_rate(10);
    while (ros::ok()) { //node_obj.nh.ok()
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
