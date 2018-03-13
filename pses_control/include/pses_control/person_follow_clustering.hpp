#ifndef PERSON_FOLLOW_CLUSTERING_HPP
#define PERSON_FOLLOW_CLUSTERING_HPP

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/PoseStamped.h"
#include <cmath>

using namespace cv;
using namespace std;

class PersonFollowClustering {

    public:
        PersonFollowClustering();
		/*
		 * NAME:			depthImageCallback
		 * DESCRIPTION:		Receives a ROS depth image and converts it to an OpenCV Mat
		 * INPUT:			const sensor_msgs::ImageConstPtr& depth_image
		 * OUTPUT: 			
		*/
		
        void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_image);
		/*
		 * NAME:			cluster
		 * DESCRIPTION:		Applies a given bounding box on the depth image and performs a clustering within
							for calculating the position of the person. Outputs a goal which is the position
							of the person shifted towards the camera
		 * INPUT:			Rect2d& bbox
							double distance_to_person
		 * OUTPUT: 			geometry_msgs::PoseStamped target - Position of person subtracted by a given distance
		*/
        geometry_msgs::PoseStamped cluster(Rect2d& bbox, double distance_to_person);

    private:
        Mat manip_depth_image_;


};

#endif // PERSON_FOLLOW_CLUSTERING_HPP
