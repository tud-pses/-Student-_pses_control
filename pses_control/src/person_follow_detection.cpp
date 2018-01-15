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
    // initialization
    String prototxt = "/home/pses/catkin_ws/src/pses_control/pses_control/data/MobileNetSSD_deploy.prototxt.txt";
    String model = "/home/pses/catkin_ws/src/pses_control/pses_control/data/MobileNetSSD_deploy.caffemodel";
    net_ = readNetFromCaffe(prototxt, model);
}

Rect2d PersonFollowDetection::detect(Mat& color_image) {
    // copy ros image to opencv image type and manipulate it
    int image_width = color_image.cols;
    int image_height = color_image.rows;
    Mat manip_image;
    Size manip_image_size(300, 300);
    resize(color_image, manip_image, manip_image_size);
    Mat blob = blobFromImage(manip_image, 0.007843, manip_image_size, 127.5);
    net_.setInput(blob);
    Mat detections = net_.forward();
    Rect2d bbox(0.0 ,0.0 ,0.0 ,0.0);
    for (int i = 0; i < detections.size[2]; ++i) {
        float confidence = detections.at<float>(Vec4i(0, 0, i, 2));
        //Mat x = detections.reshape(1,1); // 0, id, %, x, y, widht, height

        // if confidence of detected object is greater than threshold
        // get only people detetions
        // and compute bounding box
        if (confidence > confidence_thres_) {
            int idx = detections.at<float>(Vec4i(0, 0, i, 1));
            if (idx == 15) {
                Mat coords = (Mat_<float>(1,4 ) << detections.at<float>(Vec4i(0, 0, i, 3)), detections.at<float>(Vec4i(0, 0, i, 4)),
                        detections.at<float>(Vec4i(0, 0, i, 5)), detections.at<float>(Vec4i(0, 0, i, 6)));
                Mat scale = (Mat_<float>(1,4 ) << image_width, image_height, image_width, image_height);
                Mat rect(4, 1, CV_64FC1); rect = coords.mul(scale);
                rectangle(color_image, Point(rect.at<float>(0), rect.at<float>(1)), Point(rect.at<float>(2), rect.at<float>(3)), Scalar(0, 255, 0), 2);
                bbox = Rect2d(Point(rect.at<double>(0),rect.at<double>(1)),Point(rect.at<double>(2),rect.at<double>(3)));
                // publish ROI based on bounding box
                /*sensor_msgs::RegionOfInterest roi;
                roi.x_offset = bbox.at<float>(0);
                roi.y_offset = bbox.at<float>(1);
                roi.height = bbox.at<float>(3) - bbox.at<float>(1);
                roi.width = bbox.at<float>(2) - bbox.at<float>(0);
                roi.do_rectify = false;
                pub_bounding_box_.publish(roi);*/
            }
        }
    }
    if (show_) {
        imshow("Detect", color_image);
        waitKey(1);
    }

    return bbox;
}
