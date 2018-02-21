#include <ros/ros.h>
#include <cstdint>
#include "pses_control/person_follow_detection.hpp"
#include "pses_control/person_follow_tracking.hpp"
#include "pses_control/person_follow_clustering.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include "pses_control/person_followConfig.h"

PersonFollowDetection detector_;
PersonFollowTracking tracker_;
PersonFollowClustering cluster_;
Rect2d bbox_;
ros::Subscriber sub_color_image_;
ros::Subscriber sub_laser_scan_;
ros::Subscriber sub_point_cloud;
ros::Subscriber sub_depth_image;
ros::Publisher pub_scan;
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
        int height = - bbox_.y * 424.0/300.0 - bbox_.height/2.0 * 424.0/300.0 + 424.0/2.0;
        //nh_.setParam("/depthimage_to_laserscan/scan_height", height);
        int border_left = bbox_.x * 512.0/300.0;
        int border_right = border_left + bbox_.width * 512.0/300.0;
        //nh_.setParam("/depthimage_to_laserscan/scan_height", height);
        sensor_msgs::LaserScan ranges = cluster_.cluster(border_left, border_right);
        pub_scan.publish(ranges);
    }
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& points) {

    /*ROS_INFO_STREAM(points->width<< "x" << points->height << '\n' << points->point_step << '\n' << points->row_step << '\n' << points->data.size() << '\n' << points->data[0]);
    ROS_INFO_STREAM("Fieldsize: " << points->fields.size());
    for (int i = 0; i < points->fields.size(); i++) {
        ROS_INFO_STREAM("Name: " << points->fields[i].name);
        ROS_INFO_STREAM("Offset: " << points->fields[i].offset);
        ROS_INFO_STREAM("Datatype: " << points->fields[i].datatype);
        ROS_INFO_STREAM("Count: " << points->fields[i].count);
        ROS_INFO_STREAM("============================================");
    }*/

   /* int width = points->width;
    int height = points->height;
    //ROS_INFO_STREAM(width << "  " << height);
    float x[518400], y[518400], z[518400];
    int r[518400], g[518400], b[518400];
    int count = 0;
    float max_dist = 0;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int idx = i * points->row_step + j * points->point_step;
            int idx_x = idx + points->fields[0].offset;
            int idx_y = idx + points->fields[1].offset;
            int idx_z = idx + points->fields[2].offset;
            int idx_rgb = idx + points->fields[3].offset;
            float X = 0.0, Y = 0.0, Z = 0.0;
            int RGB = 0;
            memcpy(&X, &points->data[idx_x], sizeof(float));
            memcpy(&Y, &points->data[idx_y], sizeof(float));
            memcpy(&Z, &points->data[idx_z], sizeof(float));
            //memcpy(&RGB, &points->data[idx_rgb], sizeof(float));
            x[count] = X; y[count] = Y; z[count] = Z;
            if (z[count] > max_dist) max_dist = z[count];
            //r[count] = (int)(RGB && 0xFF0000);
            //g[count] = (int)(RGB && 0x00FF00);
            //b[count] = (int)(RGB && 0x0000FF);
            count++;
            //ROS_INFO_STREAM("asd");
        }
    }
    for (int i = 0; i < 518400; i++) {
        z[i] = z[i] / 7.0 * 255.0;
    }
    cv::Mat image(height, width, CV_8UC1, z);
    cv::imshow("asd", image);
    cv::waitKey(1);*/
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_image) {
    /*cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::8UC1);
        Mat manip_image;
        Size manip_image_size(300, 300);
        resize(cv_ptr->image, manip_image, manip_image_size);
        rectangle(manip_image, bbox_, Scalar( 255, 0, 0 ), 2, 1 );
        imshow("Depth_image", manip_image);
        waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error: %s", e.what());
        // return;
    }*/

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
    ROS_INFO_STREAM("reconfigure");
    cluster_.setOffset(config.offset);

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
    sub_laser_scan_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 10, &PersonFollowClustering::scanCallback, &cluster_);
    sub_point_cloud = nh_.subscribe<sensor_msgs::PointCloud2>("/kinect2/qhd/points", 10, &pointCloudCallback);
    sub_depth_image = nh_.subscribe("/kinect2/qhd/image_depth_rect", 10, &depthImageCallback);

    pub_scan = nh_.advertise<sensor_msgs::LaserScan>("/sub_scan", 1);

    ros::Rate loop_rate(10);
    while (ros::ok()) { //node_obj.nh.ok()
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
