#ifndef PERSON_FOLLOW_CLUSTERING_HPP
#define PERSON_FOLLOW_CLUSTERING_HPP

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <sensor_msgs/LaserScan.h>

using namespace cv;
using namespace std;

class PersonFollowClustering {

    public:
        PersonFollowClustering();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void cluster(Rect2d bbox);

    private:
        sensor_msgs::LaserScan scan_;

};

#endif // PERSON_FOLLOW_CLUSTERING_HPP
