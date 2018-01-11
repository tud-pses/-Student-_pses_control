#include "pses_control/person_follow_detection.hpp"

using namespace cv;
using namespace std;
using namespace dnn;

/*
 CLASSES = [ "background", "aeroplane", "bicycle", "bird", "boat",
             "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
             "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
             "sofa", "train", "tvmonitor"]
 */

PersonFollowDetection::PersonFollowDetection() {
    ros::NodeHandle params("~");
    std::string color_image_topic;
    params.param<std::string>("color_image", color_image_topic, "/kinect2/hd/image");
    params.param<bool>("show", show_, false);
    sub_color_image_ = nh.subscribe(color_image_topic, 50, &PersonFollowDetection::colorImageCallback, this);    // optimally image is resized
    pub_bounding_box_ = nh.advertise<sensor_msgs::RegionOfInterest>("detect", 1);

    // initialization
    String prototxt = "/home/pses/catkin_ws/src/pses_control/pses_control/data/MobileNetSSD_deploy.prototxt.txt";
    String model = "/home/pses/catkin_ws/src/pses_control/pses_control/data/MobileNetSSD_deploy.caffemodel";
    net_ = readNetFromCaffe(prototxt, model);
}

void PersonFollowDetection::colorImageCallback(const sensor_msgs::ImageConstPtr& color_image) {
    // copy ros image to opencv image type and manipulate it
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error: %s", e.what());
        return;
    }
    int image_width = cv_ptr->image.cols;
    int image_height = cv_ptr->image.rows;
    Mat manip_image;
    Size manip_image_size(300, 300);
    resize(cv_ptr->image, manip_image, manip_image_size);
    Mat blob = blobFromImage(manip_image, 0.007843, manip_image_size, 127.5);
    net_.setInput(blob);
    Mat detections = net_.forward();
    for (int i = 0; i < detections.size[2]; ++i) {
        float confidence = detections.at<float>(Vec4i(0, 0, i, 2));
        //Mat x = detections.reshape(1,1); // 0, id, %, x, y, widht, height

        // if confidence of detected object is greater than threshold
        // get only people detetions
        // and compute bounding box
        if (confidence > confidence_thres_) {
            int idx = detections.at<float>(Vec4i(0, 0, i, 1));
            if (idx == 15) {
                Mat rect = (Mat_<float>(1,4 ) << detections.at<float>(Vec4i(0, 0, i, 3)), detections.at<float>(Vec4i(0, 0, i, 4)),
                        detections.at<float>(Vec4i(0, 0, i, 5)), detections.at<float>(Vec4i(0, 0, i, 6)));
                Mat scale = (Mat_<float>(1,4 ) << image_width, image_height, image_width, image_height);
                Mat bbox(4, 1, CV_16UC1); bbox = rect.mul(scale);
                rectangle(cv_ptr->image, Point(bbox.at<float>(0), bbox.at<float>(1)), Point(bbox.at<float>(2), bbox.at<float>(3)), Scalar(0, 255, 0), 2);

                // publish ROI based on bounding box
                sensor_msgs::RegionOfInterest roi;
                roi.x_offset = bbox.at<float>(0);
                roi.y_offset = bbox.at<float>(1);
                roi.height = bbox.at<float>(3) - bbox.at<float>(1);
                roi.width = bbox.at<float>(2) - bbox.at<float>(0);
                roi.do_rectify = false;
                pub_bounding_box_.publish(roi);
            }
        }
    }
    if (show_) {
        imshow("Detect", cv_ptr->image);
        waitKey(1);
    }
}
