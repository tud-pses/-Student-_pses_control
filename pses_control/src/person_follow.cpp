#include <ros/ros.h>
#include <cstdint>
#include "pses_control/person_follow_detection.hpp"
#include "pses_control/person_follow_tracking.hpp"
#include "pses_control/person_follow_clustering.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include "pses_control/person_followConfig.h"

PersonFollowDetection detector_;
PersonFollowTracking tracker_;
PersonFollowClustering cluster_;
Rect2d bbox_;
ros::Subscriber sub_color_image_;
ros::Subscriber sub_point_cloud_;
ros::Subscriber sub_depth_image_;
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
    if (bbox_.tl() != bbox_.br()) {
        if(bbox_.x > 31 && bbox_.x + bbox_.width < 228) {
            if (bbox_.y < 0) bbox_.y = 0;
            if (bbox_.y + bbox_.height > 300) bbox_.height = 300 - bbox_.y;
            cluster_.cluster(bbox_);
        }
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

void reconfigureCallback(pses_control::person_followConfig &config, uint32_t level) {
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

    ros::Rate loop_rate(10);
    while (ros::ok()) { //node_obj.nh.ok()
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
