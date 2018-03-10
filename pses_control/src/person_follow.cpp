#include <ros/ros.h>
#include <cstdint>
#include "pses_control/person_follow_detection.hpp"
#include "pses_control/person_follow_tracking.hpp"
#include "pses_control/person_follow_clustering.hpp"
#include "pses_control/person_follow_control.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include "pses_control/person_followConfig.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int16.h"

PersonFollowDetection detector_;
PersonFollowTracking tracker_;
PersonFollowClustering cluster_;
PersonFollowControl control_;
Rect2d bbox_;
ros::Subscriber sub_color_image_;
ros::Subscriber sub_point_cloud_;
ros::Subscriber sub_depth_image_;
ros::Publisher pub_target_;
ros::Publisher pub_steering_;
ros::Publisher pub_velocity_;
int tracking_id_;
int cnt_ = 0;
double distance_to_person_;
ros::Time controller_timeout_;

void detectOrTrack(Mat frame) {
    ++cnt_;
    std_msgs::Int16 steering_msg, velocity_msg;
    if(bbox_.tl() == bbox_.br() || cnt_ > 100) {
        ROS_INFO_STREAM("detecting...");
        bbox_ = detector_.detect(frame);
        cnt_ = 0;
        tracker_.setInit(false);
    } else {
        ROS_INFO_STREAM("tracking...");
        bbox_ = tracker_.track(frame, bbox_);

        if(bbox_.x + 0.5*bbox_.width > 31 && bbox_.x + bbox_.width*0.5 < 228) {
            if(bbox_.x < 32) bbox_.x = 32;
            if(bbox_.x + bbox_.width > 228) bbox_.width = 228 - bbox_.x;
            if(bbox_.y < 0) bbox_.y = 0;
            if(bbox_.y + bbox_.height > 300) bbox_.height = 300 - bbox_.y;
            geometry_msgs::PoseStamped target = cluster_.cluster(bbox_, distance_to_person_);
            pub_target_.publish(target);
            float steering, velocity;
            controller_timeout_ = ros::Time::now();
            control_.control(target, steering, velocity);
            steering_msg.data = steering;
            velocity_msg.data = velocity;
        }
    }
    if (ros::Time::now().toSec() - controller_timeout_.toSec() > 3.0) {
        steering_msg.data = 0;
        velocity_msg.data = 0;
    }
    pub_steering_.publish(steering_msg);
    pub_velocity_.publish(velocity_msg);
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

void reconfigureCallback(pses_control::person_followConfig &config, uint32_t level) {
    distance_to_person_ = config.distance;
    control_.setGains(config.kp_steering, config.kd_steering, config.kp_velocity, config.kd_velocity);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "person_follow");
    ros::NodeHandle nh_;
    ros::NodeHandle params("~");
    std::string color_image;
    params.param<std::string>("image", color_image, "/kinect2/qhd/image_color");
    params.param<int>("tracker", tracking_id_, 4);
    //params.param<bool>("show", show_, false);
    dynamic_reconfigure::Server<pses_control::person_followConfig> server;
    dynamic_reconfigure::Server<pses_control::person_followConfig>::CallbackType f;
    f = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(f);
    tracker_.initTracker(tracking_id_);
    sub_color_image_ = nh_.subscribe(color_image, 10, &colorImageCallback);
    sub_point_cloud_ = nh_.subscribe("/kinect2/qhd/points", 10, &PersonFollowClustering::pointCloudCallback, &cluster_);
    sub_depth_image_ = nh_.subscribe("/kinect2/qhd/image_depth_rect", 10, &PersonFollowClustering::depthImageCallback, &cluster_);
    pub_target_ = nh_.advertise<geometry_msgs::PoseStamped>("person_target", 1);
    pub_steering_ = nh_.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);
    pub_velocity_ = nh_.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);

    ros::Rate loop_rate(10);
    while (ros::ok()) { //node_obj.nh.ok()
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
