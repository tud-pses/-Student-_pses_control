// Detects people in camera image and creates a bounding box

#include "pses_control/person_follow_detection.hpp"

using namespace cv;
using namespace std;
using namespace dnn;

PersonFollowDetection::PersonFollowDetection() {
    // Loads neural network with prototxt and model
    String prototxt = "/home/pses/catkin_ws/src/pses_control/pses_control/data/MobileNetSSD_deploy.prototxt.txt";
    String model = "/home/pses/catkin_ws/src/pses_control/pses_control/data/MobileNetSSD_deploy.caffemodel";
    net_ = readNetFromCaffe(prototxt, model);
}

// Detects person in color image
Rect2d PersonFollowDetection::detect(Mat& color_image) {
    int image_width = color_image.cols;
    int image_height = color_image.rows;
    Mat blob = blobFromImage(color_image, 0.007843, color_image.size(), 127.5);
    net_.setInput(blob);
	// Output of neural network: 0, id, %, x, y, width, height
    Mat detections = net_.forward();
	
    Rect2d bbox(0.0, 0.0, 0.0, 0.0);
    float best_confidence = 0.0;
    for (int i = 0; i < detections.size[2]; ++i) {
        float confidence = detections.at<float>(Vec4i(0, 0, i, 2));
        //Mat x = detections.reshape(1,1); 

        // if confidence of person is greater than threshold
        if (confidence > confidence_thres_) {
            int idx = detections.at<float>(Vec4i(0, 0, i, 1));
			// get person with highest confidence
            if (idx == 15 && best_confidence < confidence) {
                best_confidence = confidence;
				// coords are [0, 1]
                Mat coords = (Mat_<float>(1,4 ) << detections.at<float>(Vec4i(0, 0, i, 3)), detections.at<float>(Vec4i(0, 0, i, 4)),
                        detections.at<float>(Vec4i(0, 0, i, 5)), detections.at<float>(Vec4i(0, 0, i, 6)));
                Mat scale = (Mat_<float>(1,4 ) << image_width, image_height, image_width, image_height);
                Mat rect(4, 1, CV_64FC1);
                rect = coords.mul(scale);
				// bounding box of detected person
                bbox = Rect2d(Point(rect.at<float>(0),rect.at<float>(1)),Point(rect.at<float>(2),rect.at<float>(3)));
                // rectangle(color_image, bbox, Scalar(0, 255, 0), 2);
            }
        }
    }
	// imshow("Detect", color_image);
	// waitKey(1);
    }

    return bbox;
}
