#include "pses_control/person_follow_clustering.hpp"


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

geometry_msgs::PoseStamped PersonFollowClustering::cluster(Rect2d& bbox, double distance_to_person) {
    //cv::Mat h = (cv::Mat_<double>(3, 3) << 366.7218933105469, 0.0, 259.8772888183594, 0.0, 366.7218933105469, 203.314697265625, 0.0, 0.0, 1.0);
    cv::Mat h = (cv::Mat_<double>(3, 3) << 529.9732789120519, 0.0, 477.4416333879422, 0.0, 526.9663404399863, 261.8692914553029, 0.0, 0.0, 1.0);

    Mat bbox_image = manip_depth_image(bbox);
    for(int i =0; i < bbox_image.rows; i++) {
        for(int n=0; n < bbox_image.cols; n++) {
            if(bbox_image.at<float>(i,n)<100.0) bbox_image.at<float>(i,n)=7000.0;
        }
    }
    float mean_bbox = mean(bbox_image)[0];
    Mat mask;
    threshold(bbox_image, mask, mean_bbox, 1, THRESH_BINARY_INV);
    mask.convertTo(mask, CV_8UC1);
    float mean_distance = mean(bbox_image, mask)[0];

    double v_ = (bbox.x + bbox.width/2) * 960.0/300.0;
    double u_ = (bbox.y + bbox.height/2) * 540.0/300.0;

    cv::Mat pt = (cv::Mat_<double>(3, 1) << v_, u_, 1.0);
    pt = h.inv() * pt * mean_distance/1000.0;

    double dist_eukl = sqrt(pow(pt.at<double>(0),2)+pow(pt.at<double>(2),2));



    geometry_msgs::PoseStamped target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "camera_depth_frame";
    target.pose.position.x = pt.at<double>(0) * (1.0 - distance_to_person/dist_eukl);
    target.pose.position.y = pt.at<double>(1);
    target.pose.position.z = pt.at<double>(2) * (1.0 - distance_to_person/dist_eukl);
    target.pose.orientation.x = 0;
    target.pose.orientation.y = 0;
    target.pose.orientation.z = 0;
    target.pose.orientation.w = 1;

    //ROS_INFO_STREAM("Pixel: " << u_ << " - " << v_);
    ROS_INFO_STREAM("Abstand: " << mean_distance);
    ROS_INFO_STREAM("Koordinaten: " << target.pose.position);
    //imshow("Maske", mask*255);
    //waitKey(1);

    return target;
}
