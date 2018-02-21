#ifndef PERSON_FOLLOW_CLUSTERING_HPP
#define PERSON_FOLLOW_CLUSTERING_HPP

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>

using namespace cv;
using namespace std;

class PersonFollowClustering {

    public:
        PersonFollowClustering();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        sensor_msgs::LaserScan cluster(int left, int right);
        void setOffset(int value);


    private:
        sensor_msgs::LaserScan scan_;
        int offset_;


};

#endif // PERSON_FOLLOW_CLUSTERING_HPP
