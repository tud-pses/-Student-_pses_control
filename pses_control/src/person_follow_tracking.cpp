#include "pses_control/person_follow_tracking.hpp"

PersonFollowTracking::PersonFollowTracking() {
    // List of tracker types in OpenCV 3.2
    // NOTE : GOTURN implementation is buggy and does not work.
    string trackerTypes[6] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN"};
    // vector <string> trackerTypes(types, std::end(types));

    // Create a tracker
    string trackerType = trackerTypes[2];

    #if (CV_MINOR_VERSION < 3)
    {
        tracker = Tracker::create(trackerType);
    }
    #else
    {
        if (trackerType == "BOOSTING")
            tracker = TrackerBoosting::create();
        if (trackerType == "MIL")
            tracker = TrackerMIL::create();
        if (trackerType == "KCF")
            tracker = TrackerKCF::create();
        if (trackerType == "TLD")
            tracker = TrackerTLD::create();
        if (trackerType == "MEDIANFLOW")
            tracker = TrackerMedianFlow::create();
        if (trackerType == "GOTURN")
            tracker = TrackerGOTURN::create();
    }
    #endif
}

Rect2d PersonFollowTracking::track(Mat& frame, Rect2d& bbox) {

    if(!init) {
        tracker->init(frame, bbox);
    }
    string trackerTypes[6] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN"};
    string trackerType = trackerTypes[2];

    // Calculate Frames per second (FPS)
    double timer = (double)getTickCount();
    float fps = getTickFrequency() / ((double)getTickCount() - timer);

    init = tracker->update(frame, bbox);

    // Tracking success : Draw the tracked object
    if (init)
    {
        rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
    }
    else {
        Rect2d tmp(0, 0, 0, 0);
        bbox = tmp;
    }

    // Display tracker type on frame
    putText(frame, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);

    // Display FPS on frame
    putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

    imshow("Tracking", frame);
    waitKey(1);

    return bbox;

}
