#include "pses_control/person_follow_detection.hpp"

using namespace cv;
using namespace std;

PersonFollowDetection::PersonFollowDetection() {
    ros::NodeHandle params("~");
    std::string color_image_topic;
    params.param<std::string>("color_image", color_image_topic, "/kinect2/hd/image");
    sub_color_image_ = nh.subscribe(color_image_topic, 50, &PersonFollowDetection::colorImageCallback, this);    // optimally image is resized
    pub_bounding_box_ =nh.advertise<std_msgs::Int32MultiArray<sensor_msgs::RegionOfInterest>>("detect", 1);

    // initialization
    hog_.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
}

void PersonFollowDetection::colorImageCallback(const sensor_msgs::Image::ConstPtr& color_image) {
    // detect people in image
    vector<Rect> found, found_nms;
    hog_.detectMultiScale(color_image, found, 0, Size(4, 4), Size(8, 8), 1.05, 2);

    // apply faster non-maxima suppression
    for (int i = 0; i < found.length(); ++i) {
        found[i] = Rect(found[i].x, found[i].y, found[i].x + found[i].width, found[i].y + found[i].height);
    }
    found_nms = nms(found, 0.65);
    ROS_INFO_STREAM("People in image found: " << found_nms.length());

    // publish bounding box
    geometry_msgs::Polygon bbox;
    for (int i = 0; i < found.length(); ++i) {
        geometry_msgs::Point32(found[i].x, found[i].y, 0);          // upper left
        geometry_msgs::Point32(found[i].width, found[i].y, 0);      // upper right
        geometry_msgs::Point32(found[i].width, found[i].height, 0); // lower right
        geometry_msgs::Point32(found[i].x, found[i].height, 0);     // lower left
    }
    pub_bounding_box_.publish();
}
