// Receives a depth image and a bounding box of a person for
// performing a clustering. A coordinate of the person is calculated
// and the related goal is shifted towards the camera

#include "pses_control/person_follow_clustering.hpp"

PersonFollowClustering::PersonFollowClustering() {
}

// Receives a ROS depth image and converts it to an OpenCV Mat
void PersonFollowClustering::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
		// Converts a ROS image to a OpenCV image
        cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
        Size manip_size(300, 300);
        resize(cv_ptr->image, manip_depth_image_, manip_size);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error: %s", e.what());
        // return;
    }
}

// Applies a given bounding box on the depth image and performs a clustering within
// for calculating the position of the person.
// Outputs a goal which is the position of the person shifted towards the camera.
geometry_msgs::PoseStamped PersonFollowClustering::cluster(Rect2d& bbox, double distance_to_person) {
    cv::Mat k = (cv::Mat_<double>(3, 3) << 529.9732789120519, 0.0, 477.4416333879422, 0.0, 526.9663404399863, 261.8692914553029, 0.0, 0.0, 1.0);

	// Apply thresholding in depth image for filtering invalid pixel
    Mat bbox_image = manip_depth_image_(bbox);
    for (int i = 0; i < bbox_image.rows; i++) {
        for (int n = 0; n < bbox_image.cols; n++) {
            if (bbox_image.at<float>(i,n) < 100.0) bbox_image.at<float>(i,n) = 7000.0;
        }
    }
	// Calculate mean of all points in bounding box
    float mean_bbox = mean(bbox_image)[0];
    Mat mask;
	// Create mask by thresholding bounding box by using only points closer than mean
    threshold(bbox_image, mask, mean_bbox, 1, THRESH_BINARY_INV);
    mask.convertTo(mask, CV_8UC1);
    float mean_distance = mean(bbox_image, mask)[0];

	// gets image coordinates of middle of bounding box and scales it
    double v_ = (bbox.x + bbox.width/2) * 960.0/300.0;
    double u_ = (bbox.y + bbox.height/2) * 540.0/300.0;

	// transforms camera coordinates to world coordinates
    cv::Mat pt = (cv::Mat_<double>(3, 1) << v_, u_, 1.0);
    pt = k.inv() * pt * mean_distance/1000.0;

	// shifts world coordinates of person towards camera
    double dist_eucl = sqrt(pow(pt.at<double>(0),2)+pow(pt.at<double>(2),2));

	// creates message of target
    geometry_msgs::PoseStamped target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "camera_depth_frame";
    target.pose.position.x = pt.at<double>(0) * (1.0 - distance_to_person/dist_eucl);
    target.pose.position.y = pt.at<double>(1);
    target.pose.position.z = pt.at<double>(2) * (1.0 - distance_to_person/dist_eucl);
    target.pose.orientation.x = 0;
    target.pose.orientation.y = 0;
    target.pose.orientation.z = 0;
    target.pose.orientation.w = 1;

    //imshow("Maske", mask*255);
    //waitKey(1);

    return target;
}
