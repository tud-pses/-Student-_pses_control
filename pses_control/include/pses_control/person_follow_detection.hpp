#ifndef PERSON_FOLLOW_DETECTION_HPP
#define PERSON_FOLLOW_DETECTION_HPP

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/RegionOfInterest.h>
#include <std_msgs/String.h>
#include "pses_control/nms.cpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/version.hpp"


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
        void colorImageCallback(const sensor_msgs::Image::ConstPtr& color_image);

        // Variables
        sensor_msgs::Image color_image_;
        HOGDescriptor hog_;
};

#endif // PERSON_FOLLOW_DETECTION_HPP
