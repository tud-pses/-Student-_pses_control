#ifndef PERSON_FOLLOW_DETECTION_HPP
#define PERSON_FOLLOW_DETECTION_HPP

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/String.h>
#include "nms.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

class PersonFollowDetection {

    public:
        PersonFollowDetection();
        ros::NodeHandle nh;

    private:
        // Subscribers
        ros::Subscriber sub_color_image_;

        // Publisher
        ros::Publisher pub_bounding_box_;

        // Functions
        void colorImageCallback(const sensor_msgs::ImageConstPtr& color_image);

        // Variables
        sensor_msgs::Image color_image_;
        HOGDescriptor hog_;
};

#endif // PERSON_FOLLOW_DETECTION_HPP
