#include "pses_control/person_follow_clustering.hpp"

PersonFollowClustering::PersonFollowClustering() {

}

void PersonFollowClustering::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    scan_ = *scan;
}

void PersonFollowClustering::cluster(Rect2d bbox) {

}
