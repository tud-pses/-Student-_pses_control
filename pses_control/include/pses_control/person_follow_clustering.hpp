#ifndef PERSON_FOLLOW_CLUSTERING_HPP
#define PERSON_FOLLOW_CLUSTERING_HPP

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseStamped.h"
#include <cmath>

using namespace cv;
using namespace std;

class PersonFollowClustering {

    public:
        PersonFollowClustering();
        void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_image);
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& points);
        geometry_msgs::PoseStamped cluster(Rect2d& bbox, double distance_to_person);

    private:
        Mat manip_depth_image;


};

#endif // PERSON_FOLLOW_CLUSTERING_HPP
