// Tracks a person by supplying an initialization bounding box

#include "pses_control/person_follow_tracking.hpp"

PersonFollowTracking::PersonFollowTracking() {

}

// Initializes the tracker by a given ID
void PersonFollowTracking::initTracker(int tracking_id) {
    tracking_id_ = tracking_id;
    string trackerTypes[6] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN"};
    string trackerType = trackerTypes[tracking_id];

    #if (CV_MINOR_VERSION < 3)
    {
        tracker_ = Tracker::create(trackerType);
    }
    #else
    {
        if (trackerType == "BOOSTING")
            tracker_ = TrackerBoosting::create();
        if (trackerType == "MIL")
            tracker_ = TrackerMIL::create();
        if (trackerType == "KCF")
            tracker_ = TrackerKCF::create();
        if (trackerType == "TLD")
            tracker_ = TrackerTLD::create();
        if (trackerType == "MEDIANFLOW")
            tracker_ = TrackerMedianFlow::create();
        if (trackerType == "GOTURN")
            tracker_ = TrackerGOTURN::create();
    }
    #endif
}


// Sets global variable if tracker is initialized
void PersonFollowTracking::setInit(bool arg0) {
    init_ = arg0;
}

// Tracks content of given bounding box in given image
Rect2d PersonFollowTracking::track(Mat& frame, Rect2d& bbox) {

    if(!init_) {
        initTracker(tracking_id_);
        tracker_->init(frame, bbox);
    }

    // Calculate Frames per second (FPS)
    double timer = (double)getTickCount();
    init_ = tracker_->update(frame, bbox);
    float fps = getTickFrequency() / ((double)getTickCount() - timer);

    // Tracking success: Draw the tracked object
    if (init_)
    {
        // rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
    }
    else {
        Rect2d tmp(0, 0, 0, 0);
        bbox = tmp;
    }
	
    // imshow("Tracking", frame);
    // waitKey(1);
    return bbox;

}
