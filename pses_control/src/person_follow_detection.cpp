#include "pses_control/person_follow_detection.hpp"

using namespace cv;
using namespace std;

PersonFollowDetection::PersonFollowDetection() {
    ros::NodeHandle params("~");
    std::string color_image_topic;
    params.param<std::string>("color_image", color_image_topic, "/kinect2/hd/image");
    sub_color_image_ = nh.subscribe(color_image_topic, 50, &PersonFollowDetection::colorImageCallback, this);    // optimally image is resized
    pub_bounding_box_ = nh.advertise<sensor_msgs::RegionOfInterest>("detect", 1);

    // initialization
    hog_.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
}

void PersonFollowDetection::colorImageCallback(const sensor_msgs::ImageConstPtr& color_image) {
    // copy ros image to opencv image type
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(color_image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error: %s", e.what());
        return;
    }

    // detect people in image
    vector<Rect> found, rects_nms; // x, y, width, height
    hog_.detectMultiScale(cv_ptr->image, found, 0, Size(4, 4), Size(8, 8), 1.05, 2, false);

    // apply faster non-maxima suppression
    vector<vector<float>> rects;
    for (int i = 0; i < found.size(); ++i) {
        vector<float> rect;
        rect.push_back(found[i].x);
        rect.push_back(found[i].y);
        rect.push_back(found[i].x + found[i].width);
        rect.push_back(found[i].y + found[i].height);
    }
    rects_nms = nms(rects, 0.65);
    ROS_INFO_STREAM("People in image found: " << rects_nms.size());

    // publish bounding box
    for (int i = 0; i < rects_nms.size(); ++i) {
        sensor_msgs::RegionOfInterest roi;
        roi.x_offset = rects_nms[i].x;
        roi.y_offset = rects_nms[i].y;
        roi.height = rects_nms[i].height - rects_nms[i].y;
        roi.width = rects_nms[i].width - rects_nms[i].x;
        pub_bounding_box_.publish(roi);

        rectangle(cv_ptr->image, Point(roi.x_offset, roi.y_offset + roi.height), Point(roi.x_offset + roi.width, roi.y_offset), Scalar(255, 0 ,0), 2);
    }
    imshow("OpenCV Video", cv_ptr->image);
    waitKey(1);
}
