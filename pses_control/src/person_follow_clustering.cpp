#include "pses_control/person_follow_clustering.hpp"
#include <cmath>

PersonFollowClustering::PersonFollowClustering() {
    offset_ = 0;

}

void PersonFollowClustering::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    scan_ = *scan;
}

void PersonFollowClustering::setOffset(int value) {
    offset_ = value;
}

sensor_msgs::LaserScan PersonFollowClustering::cluster(int left, int right) {
    //std_msgs::Float32MultiArray ranges;
    int idx = 0;
    /*for (int i = left; i < right + 1; i++) {
        ranges.data.push_back(scan_.ranges[i]);
        ++idx;
    }
    return ranges;*/

    sensor_msgs::LaserScan sub_scan(scan_);


    //Test
    int left_new = scan_.ranges.size()-right;
    int right_new = scan_.ranges.size()-left;

    //ROS_INFO_STREAM(scan_.angle_min << scan_.angle_max);
    //ROS_INFO_STREAM(scan_.angle_increment);
    ROS_INFO_STREAM(left << " - " << right);
    for (int i = 0; i < scan_.ranges.size(); i++) {
        if (i < left_new - offset_ || i > right_new - offset_)
            sub_scan.ranges[i]=7.0;
    }
    return sub_scan;
}
