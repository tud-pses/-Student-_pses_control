#ifndef PERSON_FOLLOW_DETECTION_HPP
#define PERSON_FOLLOW_DETECTION_HPP

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/dnn.hpp>

using namespace cv;
using namespace std;
using namespace dnn;

class PersonFollowDetection {

    public:
        PersonFollowDetection();
        // Functions
		
        /*
         * NAME:        detect
         * DESCRIPTION:	Detects person in color image
         * INPUT:       Mat& color_image
         * OUTPUT:      Rect2d bbox - Bounding box of detected person in given image
        */
        Rect2d detect(Mat& color_image);

    private:
        // Publisher

        // Variables
        Net net_;
        float confidence_thres_ = 0.8;
};

#endif // PERSON_FOLLOW_DETECTION_HPP
