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
    /*
    ros::NodeHandle params("~");
    params.param<bool>("show", show_, false);
    */
    // initialization
    String prototxt = "/home/garwin/catkin_ws/src/pses_control/pses_control/data/MobileNetSSD_deploy.prototxt.txt";
    String model = "/home/garwin/catkin_ws/src/pses_control/pses_control/data/MobileNetSSD_deploy.caffemodel";
    net_ = readNetFromCaffe(prototxt, model);
}

Rect2d PersonFollowDetection::detect(Mat& color_image) {
    // copy ros image to opencv image type and manipulate it
    int image_width = color_image.cols;
    int image_height = color_image.rows;
    Mat blob = blobFromImage(color_image, 0.007843, color_image.size(), 127.5);
    net_.setInput(blob);
    Mat detections = net_.forward();
    Rect2d bbox(0.0, 0.0, 0.0, 0.0);
    float best_confidence = 0.0;
    //ROS_INFO_STREAM(detections.reshape(1, 1));

    for (int i = 0; i < detections.size[2]; ++i) {
        float confidence = detections.at<float>(Vec4i(0, 0, i, 2));
        //Mat x = detections.reshape(1,1); // 0, id, %, x, y, widht, height

        // if confidence of detected object is greater than threshold
        // get only people detetions
        // and compute bounding box
        if (confidence > confidence_thres_) {
            ROS_INFO_STREAM(confidence);
            int idx = detections.at<float>(Vec4i(0, 0, i, 1));
            if (idx == 15 && best_confidence < confidence) {
                best_confidence = confidence;
                Mat coords = (Mat_<float>(1,4 ) << detections.at<float>(Vec4i(0, 0, i, 3)), detections.at<float>(Vec4i(0, 0, i, 4)),
                        detections.at<float>(Vec4i(0, 0, i, 5)), detections.at<float>(Vec4i(0, 0, i, 6)));
                Mat scale = (Mat_<float>(1,4 ) << image_width, image_height, image_width, image_height);
                Mat rect(4, 1, CV_64FC1);
                rect = coords.mul(scale);
                bbox = Rect2d(Point(rect.at<float>(0),rect.at<float>(1)),Point(rect.at<float>(2),rect.at<float>(3)));
                rectangle(color_image, bbox, Scalar(0, 255, 0), 2);
                ROS_INFO_STREAM(bbox);
            }
        }
    }

    if (true) {
        /*int x_new = bbox.x * 960.0/300.0;
        int height_new = bbox.height * 540.0/300.0;
        int y_new = bbox.y * 540.0/300.0;
        int width_new = bbox. width * 960.0/300.0;
        Rect2d bbox_new(x_new, y_new, width_new, height_new);
        Mat manip_image;
        resize(color_image, manip_image, Size(960,540));
        rectangle(manip_image, bbox_new, Scalar(0, 255, 0), 2);*/

        imshow("Detect", color_image);
        waitKey(1);
    }

    return bbox;
}
