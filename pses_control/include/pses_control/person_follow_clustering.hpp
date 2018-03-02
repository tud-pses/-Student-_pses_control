#ifndef PERSON_FOLLOW_CLUSTERING_HPP
#define PERSON_FOLLOW_CLUSTERING_HPP

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

using namespace cv;
using namespace std;

class PersonFollowClustering {

    public:
        PersonFollowClustering();
        void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_image);
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& points);
        void cluster(Rect2d& bbox);

    private:
        Mat manip_depth_image;

};

#endif // PERSON_FOLLOW_CLUSTERING_HPP
