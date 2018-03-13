#ifndef PERSON_FOLLOW_TRACKING_HPP
#define PERSON_FOLLOW_TRACKING_HPP

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/core/ocl.hpp"

using namespace cv;
using namespace std;

class PersonFollowTracking {

    public:
        PersonFollowTracking();
		
        /*
         * NAME:        track
         * DESCRIPTION: Tracks content of given bounding box in given image
         * INPUT:       Mat& frame - Color image
                        Rect2d& bbox - Initialization bounding box from detector
         * OUTPUT:      Rect2d bbox - Bounding box of person in given image
        */
        Rect2d track(Mat& frame, Rect2d& bbox);
		
        /*
         * NAME:        initTracker
         * DESCRIPTION:	Initializes the tracker by a given ID
         * INPUT:       int tracking_id
         * OUTPUT:      void
        */
        void initTracker(int tracking_id);
		
        /*
         * NAME:        setInit
         * DESCRIPTION: Sets global variable if tracker is initialized
         * INPUT:       bool arg0 - False if initialization is necessary
         * OUTPUT:      void
        */
        void setInit(bool arg0);

    private:
        Ptr<Tracker> tracker_;
        bool init_ = false;
        int tracking_id_;

};

#endif // PERSON_FOLLOW_TRACKING_HPP
