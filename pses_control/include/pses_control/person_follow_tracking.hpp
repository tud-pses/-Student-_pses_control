#ifndef PERSON_FOLLOW_TRACKING_HPP
#define PERSON_FOLLOW_TRACKING_HPP

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/core/ocl.hpp"

using namespace cv;
using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

class PersonFollowTracking {

    public:
        PersonFollowTracking();
        Rect2d track(Mat& frame, Rect2d& bbox);

    private:
        Ptr<Tracker> tracker;
        bool init;

};

#endif // PERSON_FOLLOW_TRACKING_HPP
