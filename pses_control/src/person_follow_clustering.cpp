#include "pses_control/person_follow_clustering.hpp"
#include <cmath>

PersonFollowClustering::PersonFollowClustering() {
}

void PersonFollowClustering::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
        Size manip_size(300, 300);
        resize(cv_ptr->image, manip_depth_image, manip_size);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error: %s", e.what());
        // return;
    }
}

void PersonFollowClustering::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& points) {

}

void PersonFollowClustering::cluster(Rect2d& bbox) {
    Mat bbox_image = manip_depth_image(bbox);
    float mean_bbox = mean(bbox_image)[0];
    Mat mask;
    threshold(bbox_image, mask, mean_bbox, 1, THRESH_BINARY_INV);
    mask.convertTo(mask, CV_8UC1);
    float mean_distance = mean(bbox_image, mask)[0];
}
